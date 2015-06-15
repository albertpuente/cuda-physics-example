/*
Compile with: 

    gcc physics.c -o physics -O2 -lm -std=c99 

    -O2      Optimization
    -lm      Link to math lib
    -std=c99 Use of for(;;;) with declaration among other things

Usage (3D viewer):

    ./physics > data && ./physicsViewer data
    
Usage (debug):

    ./physics

Jan Mas Rovira
Andrés Mingorance López
Albert Puente Encinas
*/

#include <stdio.h>  // e.g. printf
#include <stdlib.h> // e.g. malloc, RAND_MAX, exit
#include <math.h>   // e.g. sin, abs
#include <sys/time.h>
#include <cuda.h>

// Algorithm parameters
#define N 256*16 // Si incrementa FALLA (+110 MB)!
#define ITERATIONS 2000
#define G 9.81
#define BOUNCE_DECAY 0.5
#define GLOBAL_DECAY 0.004
#define POINT_RADIUS 0.3
#define TIME_SPEED 0.0075
#define MAX_TRIES 1e4
#define SEED 27

#define DUMP_RATIO 2

// CUDA Variables
#define nThreads 1024
#define nBlocks N/nThreads

// c++ style
#define bool int
#define true 1
#define false 0

#define WALLS true

// Timers
unsigned long long initialGenTime;
unsigned long long interactionsTime;
unsigned long long worldInteractionsTime;
unsigned long long gravityTime;
unsigned long long advanceTime;
unsigned long long frameTime;
unsigned long long totalTime;

inline void tic(unsigned long long* time) {
    struct timeval t;
    gettimeofday(&t, NULL);
    *time = t.tv_sec*1000000 + t.tv_usec - *time;
    
}
#define toc tic

// Output toggles
bool DUMP;

typedef struct {
    float x, y, z;
} Vector;

typedef struct {
    float x, y, z;      // Position
    Vector velocity;    // Velocity
} Point;

typedef struct {
    Point points[N];
} PointSet;

void checkCudaError(char msg[]) {
    cudaError_t error;
    error = cudaGetLastError();
    if (error) {
        printf("Error: %s: %s\n", msg, cudaGetErrorString(error));
        exit(1);
    }
}

inline float dist(Point* a, Point* b) {
    return sqrt(pow(a->x - b->x, 2)+pow(a->y - b->y, 2)+pow(a->z - b->z, 2));
}

__device__ inline float gpu_dist(Point* a, Point* b) {
    return sqrt(pow(a->x - b->x, 2)+pow(a->y - b->y, 2)+pow(a->z - b->z, 2));
}

__device__ inline float distNext(Point* a, Point* b) {
    return sqrt( pow(a->x + a->velocity.x*TIME_SPEED - (b->x + b->velocity.x*TIME_SPEED), 2)+
                 pow(a->y + a->velocity.y*TIME_SPEED - (b->y + b->velocity.y*TIME_SPEED), 2)+
                 pow(a->z + a->velocity.z*TIME_SPEED - (b->z + b->velocity.z*TIME_SPEED), 2));
}

bool collides(Point* p, PointSet* PS, int from, int to) {
    for (int i = from; i < to; ++i) {
        if (dist(p, &PS->points[i]) < POINT_RADIUS*2) {
            return true;
        }
    }
    return false;
}

__device__ Vector diffVector(Point* a, Point* b) {
    Vector v;
    v.x = a->x - b->x;
    v.y = a->y - b->y;
    v.z = a->z - b->z;
    return v;
}

__device__ inline float dotProduct(Vector a, Vector b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}


__global__ void kernel_interaction(PointSet* P, PointSet* Q) {
    int index = blockIdx.x * blockDim.x + threadIdx.x;    
    
    int j = 1 + floor(-0.5 + sqrt(0.25 + 2 * index));
    int triangularNumber = j * (j - 1) / 2;
    int i = index - triangularNumber;
    
    Point* a = &P->points[i];
    Point* b = &P->points[j];

    float distance = gpu_dist(a, b);
    
    if (distance > 2*POINT_RADIUS + 0.05) return;
    
    if (distance == 0) return; // AVOID NAN, PROVISIONAL
    
    if (distance < distNext(a, b)) return;
    
    Point* aq = &Q->points[i];
    Point* bq = &Q->points[j];
    
    // Get the components of the velocity vectors which are parallel to the collision.
    // The perpendicular component remains the same for both fish
    Vector collision = diffVector(a, b);
    
    //
    //distance = 2*POINT_RADIUS;
    collision.x /= distance;
    collision.y /= distance;
    collision.z /= distance;
    
    float aci = dotProduct(collision, a->velocity); 
    float bci = dotProduct(collision, b->velocity); 

    // Replace the collision velocity components with the new ones

    atomicAdd(&aq->velocity.x, (bci - aci) * collision.x);
    atomicAdd(&aq->velocity.y, (bci - aci) * collision.y);
    atomicAdd(&aq->velocity.z, (bci - aci) * collision.z);
        
    atomicAdd(&bq->velocity.x, (aci - bci) * collision.x);
    atomicAdd(&bq->velocity.y, (aci - bci) * collision.y);
    atomicAdd(&bq->velocity.z, (aci - bci) * collision.z);
}

void computeInteraction(PointSet* gpu_P, PointSet* gpu_Q) {   
    /*
     * (n*(n-1))/2 elements
     * because we are emulating the following loop
     * 
     * for (int i = 0; i < N; ++i)
     *     for (int j = i + 1; j < N; ++j)
     *         kernel_interaction(i, j);
     */
    // nThreads is 1024
    
    int nElem = (N*(N-1))/2;
    int nElemBlocks = nElem/nThreads;
    kernel_interaction<<<nElemBlocks, nThreads>>>(gpu_P, gpu_Q);
    checkCudaError((char *) "kernel call in interaction");    
    cudaDeviceSynchronize();
}

__global__ void kernel_gravity(PointSet* P) {
    int id = blockIdx.x * blockDim.x + threadIdx.x;
    P->points[id].velocity.y -= G*TIME_SPEED;    
}

void applyGravity(PointSet* gpu_P) {
    kernel_gravity<<<nBlocks, nThreads>>>(gpu_P);
    checkCudaError((char *) "kernel call in applyGravity");    
    cudaDeviceSynchronize();
}

__global__ void kernel_advance(PointSet* P, PointSet* Q) {
    int id = blockIdx.x * blockDim.x + threadIdx.x;
    Point* p = &P->points[id];
    p->x += p->velocity.x*TIME_SPEED;
    p->y += p->velocity.y*TIME_SPEED;
    p->z += p->velocity.z*TIME_SPEED;
    p->velocity.x *= (1-GLOBAL_DECAY);
    p->velocity.y *= (1-GLOBAL_DECAY);
    p->velocity.z *= (1-GLOBAL_DECAY);
    
    Q->points[id] = *p;
}

void advanceAndCopy(PointSet* gpu_P, PointSet* gpu_Q) {
    kernel_advance<<<nBlocks, nThreads>>>(gpu_P, gpu_Q);
    checkCudaError((char *) "kernel call in advance");    
    cudaDeviceSynchronize();
}

__device__ inline void ifelse(bool condition, float* dest, float a, float b) {
    *dest = condition*a + !condition*b;    
}

__global__ void kernel_world(PointSet* P) {
    int id = blockIdx.x * blockDim.x + threadIdx.x;
    
    Point* p = &P->points[id];
    if (p->y < POINT_RADIUS) {
        p->y = POINT_RADIUS;
        p->velocity.y = abs(p->velocity.y) * (1.0 - BOUNCE_DECAY);
    }
    
    if (WALLS) { // 4 walls x = -10, 10 and z = -10, 10
        if (p->x < -10.0 + POINT_RADIUS) {
            p->x = -10 + POINT_RADIUS;
            p->velocity.x = abs(p->velocity.x) * (1.0 - BOUNCE_DECAY);
        }
        else if (p->x > 10.0 - POINT_RADIUS) {
            p->x = 10 - POINT_RADIUS;
            p->velocity.x = -abs(p->velocity.x) * (1.0 - BOUNCE_DECAY);
        }            
        
        if (p->z < -10.0 + POINT_RADIUS) {
            p->z = -10 + POINT_RADIUS;
            p->velocity.z = abs(p->velocity.z) * (1.0 - BOUNCE_DECAY);
        }            
        else if (p->z > 10.0 - POINT_RADIUS) {
            p->z = 10 - POINT_RADIUS;
            p->velocity.z = -abs(p->velocity.z) * (1.0 - BOUNCE_DECAY);
        }
        
    }
}

void computeInteractionWorld(PointSet* gpu_P) {
    kernel_world<<<nBlocks, nThreads>>>(gpu_P);
    checkCudaError((char *) "kernel call in interactionWorld");    
    cudaDeviceSynchronize();
}


void computePhysics(PointSet* gpu_P, PointSet* gpu_Q) {
    tic(&gravityTime);
    applyGravity(gpu_Q); 
    toc(&gravityTime);
    
    tic(&worldInteractionsTime);
    computeInteractionWorld(gpu_Q);
    tic(&worldInteractionsTime);
    
    tic(&interactionsTime);
    computeInteraction(gpu_P, gpu_Q);
    tic(&interactionsTime);
    
    tic(&advanceTime);
    advanceAndCopy(gpu_Q, gpu_P);
    toc(&advanceTime);
}

void generateInitialConfiguration(PointSet* gpu_P, PointSet* gpu_Q) {
    tic(&initialGenTime);
    PointSet* P = (PointSet*) malloc(sizeof(PointSet));
    
    for (int i = 0; i < N; ++i) {
        
        Point* p = &P->points[i]; 

        p->x = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;
        p->y = 80.0*(float)rand()/(float)(RAND_MAX) + 1.0;
        p->z = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;       
        
        p->velocity.x = 0.0;
        p->velocity.y = -3.5;
        p->velocity.z = 0.0;
                
        int tests = 0;
        while (tests < MAX_TRIES && collides(p, P, 0, i)) {

            p->x = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;
            p->y = 80.0*(float)rand()/(float)(RAND_MAX) + 1.0;
            p->z = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;       
            ++tests;
        }
        if (tests == MAX_TRIES) {
            printf("Error during the generation of the initial conf.\n");
            exit(1);
        }
    }
    
    cudaMemcpy(gpu_P, P, sizeof(PointSet), cudaMemcpyHostToDevice);
    checkCudaError((char *) "host -> gpu_P");
    
    cudaMemcpy(gpu_Q, P, sizeof(PointSet), cudaMemcpyHostToDevice);
    checkCudaError((char *) "host -> gpu_Q");
    
    cudaDeviceSynchronize();
    
    toc(&initialGenTime);
}

void DUMPInitialParams() {
    printf("%i %i %f\n", N, ITERATIONS/DUMP_RATIO, POINT_RADIUS);
}

__global__ void kernel_print(PointSet* P) {
     int id = blockIdx.x * blockDim.x + threadIdx.x;
     printf("%f %f %f\n", P->points[id].x, P->points[id].y, P->points[id].z);
}

void dump(PointSet* gpu_P) {
    kernel_print<<<nBlocks, nThreads>>>(gpu_P);
    checkCudaError((char *) "kernel call in interaction");    
    cudaDeviceSynchronize();
}

void initTimes() {
    initialGenTime = 0;
    interactionsTime = 0;
    worldInteractionsTime = 0;
    gravityTime = 0;
    advanceTime = 0;
    frameTime = 0;
    totalTime = 0;
}

void printTimes() {
    printf("CUDA physics algorithm has finished:\n");
    printf("    Init gen:     %f s.\n", (double)initialGenTime/1000000);
    printf("    Interactions: %f s.\n", (double)interactionsTime/1000000);
    printf("    World int.:   %f s.\n", (double)worldInteractionsTime/1000000);
    printf("    Gravity:      %f s.\n", (double)gravityTime/1000000);
    printf("    Advance:      %f s.\n", (double)advanceTime/1000000);
    // printf("    Avg. frame:   %f s.\n", (double)frameTime/(ITERATIONS*1000000));
    printf("    Total time:   %f s.\n", (double)totalTime/1000000);
}

void sequentialPhysics() {
    
    DUMPInitialParams();  
        
    PointSet* gpu_P;
    PointSet* gpu_Q;
    cudaMalloc((void **) &gpu_P, sizeof(PointSet));
    checkCudaError((char *) "cudaMalloc of P");
    cudaMalloc((void **) &gpu_Q, sizeof(PointSet));
    checkCudaError((char *) "cudaMalloc of Q");
    
    tic(&totalTime);
    srand(SEED);
    generateInitialConfiguration(gpu_P, gpu_Q); // *CPU_P = *gpu_P = *gpu_Q
    
    
    for (int i = 0; i < ITERATIONS; ++i) {
        tic(&frameTime);
        computePhysics(gpu_P, gpu_Q);      
        if (DUMP) {
            if (i%DUMP_RATIO == 0) dump(gpu_P);
        }
        else printf("IT %i\n", i);
        
        toc(&frameTime);    
    }
    
    toc(&totalTime);
    if (!DUMP) printTimes();   
}

int main(int argc, char** argv) {
    DUMP = (argc == 1);
    sequentialPhysics();
    return 0;
}

