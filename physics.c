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

// c++ style
typedef int bool;
#define true 1
#define false 0

#define WALLS true

// Timers
unsigned long long initialGenTime;
unsigned long long interactionsTime;
unsigned long long worldInteractionsTime;
unsigned long long gravityTime;
unsigned long long advanceTime;
unsigned long long copyTime;
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

inline float dist(Point* a, Point* b) {
    return sqrt(pow(a->x - b->x, 2)+pow(a->y - b->y, 2)+pow(a->z - b->z, 2));
}

inline float distNext(Point* a, Point* b) {
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

Vector diffVector(Point* a, Point* b) {
    Vector v;
    v.x = a->x - b->x;
    v.y = a->y - b->y;
    v.z = a->z - b->z;
    return v;
}

inline float dotProduct(Vector a, Vector b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

void computeInteraction(PointSet* P, PointSet* Q, int i, int j) {

    if (i == j) return;
    
    Point* a = &P->points[i];
    Point* b = &P->points[j];
    
    float distance = dist(a, b);
    
    if (distance > 2*POINT_RADIUS + 0.05) return;
    
    if (distance == 0) return; // AVOID NAN, PROVISIONAL
    
    if (distance < distNext(a, b)) return;
    
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

    // Solve for the new velocities using the 1-dimensional elastic collision equations.
    // Turns out it's really simple when the masses are the same.
    float acf = bci;
    float bcf = aci;

    // Replace the collision velocity components with the new ones
    Point* aq = &Q->points[i];
    Point* bq = &Q->points[j];
    aq->velocity.x += (acf - aci) * collision.x;
    aq->velocity.y += (acf - aci) * collision.y;
    aq->velocity.z += (acf - aci) * collision.z;
    
    bq->velocity.x += (bcf - bci) * collision.x;
    bq->velocity.y += (bcf - bci) * collision.y;
    bq->velocity.z += (bcf - bci) * collision.z;

}

void applyGravity(PointSet* P) {
    for (int i = 0; i < N; ++i) {
        Point* p = &P->points[i];
        p->velocity.y -= G*TIME_SPEED;
    }
}

void advance(PointSet* P) {
    for (int i = 0; i < N; ++i) {
        Point* p = &P->points[i];
        p->x += p->velocity.x*TIME_SPEED;
        p->y += p->velocity.y*TIME_SPEED;
        p->z += p->velocity.z*TIME_SPEED;
        p->velocity.x *= (1-GLOBAL_DECAY);
        p->velocity.y *= (1-GLOBAL_DECAY);
        p->velocity.z *= (1-GLOBAL_DECAY);
    }
}

void computeInteractionWorld(PointSet* P) {
    for (int i = 0; i < N; ++i) {
        // REVISAR
        Point* p = &P->points[i];
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
}



void swap_ps(PointSet** P, PointSet** Q) {
    PointSet* aux;
    aux = *P;
    *P = *Q;
    *Q = aux;    
}

void computePhysics(PointSet* P, PointSet* Q) {
    tic(&gravityTime);
    applyGravity(Q); 
    toc(&gravityTime);
    
    tic(&worldInteractionsTime);
    computeInteractionWorld(Q);
    tic(&worldInteractionsTime);
    
    tic(&interactionsTime);
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            computeInteraction(P, Q, i, j);
        }
    }
    tic(&interactionsTime);
    
    tic(&advanceTime);
    advance(Q);
    toc(&advanceTime);
    
    // Idea: swap of pointers and position copy while computeInteraction on "i"
    tic(&copyTime); 
    *P = *Q;
    toc(&copyTime);
}

void generateInitialConfiguration(PointSet* P) {
    tic(&initialGenTime);
        
    for (int i = 0; i < N; ++i) {
        
        Point* p = &P->points[i]; 

        p->x = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;
        p->y = 20.0*(float)rand()/(float)(RAND_MAX) + 1.0;
        p->z = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;       
        
        p->velocity.x = 0.0;
        p->velocity.y = -3.5;
        p->velocity.z = 0.0;
                
        int tests = 0;
        while (tests < MAX_TRIES && collides(p, P, 0, i)) {

            p->x = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;
            p->y = 20.0*(float)rand()/(float)(RAND_MAX) + 1.0;
            p->z = 12.0*(float)rand()/(float)(RAND_MAX) - 6.0;       
            ++tests;
        }
        if (tests == MAX_TRIES) {
            printf("Error during the generation of the initial conf.\n");
            exit(1);
        }
    }
    
    toc(&initialGenTime);
}

void DUMPInitialParams() {
    printf("%i %i %f\n", N, ITERATIONS/DUMP_RATIO, POINT_RADIUS);
}

void dump(PointSet* P) {
    for (int i = 0; i < N; ++i) {
        printf("%f %f %f\n", P->points[i].x, P->points[i].y, P->points[i].z);
    }
}

void initTimes() {
    initialGenTime = 0;
    interactionsTime = 0;
    worldInteractionsTime = 0;
    gravityTime = 0;
    advanceTime = 0;
    copyTime = 0;
    frameTime = 0;
    totalTime = 0;
}

void printTimes() {
    printf("Sequential physics algorithm has finished:\n");
    printf("    Init gen:     %f s.\n", (double)initialGenTime/1000000);
    printf("    Interactions: %f s.\n", (double)interactionsTime/1000000);
    printf("    World int.:   %f s.\n", (double)worldInteractionsTime/1000000);
    printf("    Gravity:      %f s.\n", (double)gravityTime/1000000);
    printf("    Advance:      %f s.\n", (double)advanceTime/1000000);
    printf("    Copy:         %f s.\n", (double)copyTime/1000000);
    printf("    Avg. frame:   %f s.\n", (double)frameTime/(ITERATIONS*1000000));
    printf("    Total time:   %f s.\n", (double)totalTime/1000000);
}

void sequentialPhysics() {
    tic(&totalTime);   
    
    DUMPInitialParams();
    
    PointSet* P = malloc(sizeof(PointSet));
    PointSet* Q = malloc(sizeof(PointSet));
    
    srand(SEED);
    generateInitialConfiguration(P);
    *Q = *P;
    for (int i = 0; i < ITERATIONS; ++i) {
        tic(&frameTime);        
        computePhysics(P, Q);      
        if (DUMP) {
            if (i%DUMP_RATIO == 0) dump(P);
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

