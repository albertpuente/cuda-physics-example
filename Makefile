all: physics cuda # physicsViewer

# CUDA
CUDA_HOME   = /Soft/cuda/6.5.14
NVCC        = $(CUDA_HOME)/bin/nvcc
NVCC_FLAGS  = -O3 -I$(CUDA_HOME)/include -arch=compute_35 -code=sm_35 -rdc=true -I$(CUDA_HOME)/sdk/CUDALibraries/common/inc 
LD_FLAGS    = -lcudadevrt -Xlinker -rpath,$(CUDA_HOME)/lib64 -I$(CUDA_HOME)/sdk/CUDALibraries/common/lib


# C
CFLAGS=-O2 -lm -std=c99 -w
GFLAGS=-lGL -lglut -lGLU

physics: physics.c
	gcc physics.c -o physics $(CFLAGS)
	
physicsViewer: physicsViewer.c
	gcc physicsViewer.c -o physicsViewer $(CFLAGS) $(GFLAGS) 
	
debug:
	gcc -ggdb physicsViewer.c -o physicsViewer $(CFLAGS) $(GFLAGS)
	gdb physicsViewer
	
show:	physics physicsViewer
	./physics > data 
	./physicsViewer data

exec:	physics
	./physics 1
	
berry:	cuda
	./physicsCUDA 1
	
clean:
	rm -f *.o physics physicsCUDA PHYSICS* physicsViewer 
	
cuda.o: physics.cu
	$(NVCC) -c -o $@ physics.cu $(NVCC_FLAGS)

cuda:	cuda.o
	$(NVCC) cuda.o -o physicsCUDA $(LD_FLAGS)

sub:	cuda
	rm -f SESION*
	qsub -l cuda job.sh && watch -n 0.5 qstat
	
subseq: physics
	qsub -l cuda jobseq.sh && watch -n 0.5 qstat
