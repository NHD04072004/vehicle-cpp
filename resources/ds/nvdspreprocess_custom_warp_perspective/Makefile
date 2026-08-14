CUDA_VER?=
ifeq ($(CUDA_VER),)
$(error "CUDA_VER is not set")
endif

CXX:= g++
NVCC:= /usr/local/cuda-$(CUDA_VER)/bin/nvcc
NVDS_VERSION:=8.0

TARGET_LIB:= libnvdspreprocess_custom_warp_perspective.so

DS_SRC:= /opt/nvidia/deepstream/deepstream-$(NVDS_VERSION)
DS_INC:= /opt/nvidia/deepstream/deepstream/sources
LIB_INSTALL_DIR?= $(DS_SRC)/lib/

CPP_SRCS:= nvdspreprocess_warp_perspective.cpp
CU_SRCS:= warp_kernels.cu
CPP_OBJS:= $(CPP_SRCS:.cpp=.o)
CU_OBJS:= $(CU_SRCS:.cu=.o)
OBJS:= $(CPP_OBJS) $(CU_OBJS)

CFLAGS:= -fPIC -std=c++17 -Wall -Wno-error=deprecated-declarations \
	-I /usr/local/cuda-$(CUDA_VER)/include \
	-I $(DS_INC)/gst-plugins/gst-nvdspreprocess/include \
	-I $(DS_INC)/includes

NVCCFLAGS:= -Xcompiler -fPIC -std=c++17 -O2 \
	-I /usr/local/cuda-$(CUDA_VER)/include \
	-gencode arch=compute_86,code=sm_86 \
	-gencode arch=compute_75,code=sm_75 \
	-gencode arch=compute_89,code=sm_89

PKGS:= gstreamer-1.0

CFLAGS+= $(shell pkg-config --cflags $(PKGS))

LIBS:= -shared -Wl,-no-undefined \
	-L/usr/local/cuda-$(CUDA_VER)/lib64 -lcudart \
	-L$(LIB_INSTALL_DIR) -lnvbufsurface -lnvbufsurftransform -lnvds_meta -lnvdsgst_meta \
	-lcuda -Wl,-rpath,$(LIB_INSTALL_DIR)

LIBS+= $(shell pkg-config --libs $(PKGS))

all: $(TARGET_LIB)

%.o: %.cpp nvdspreprocess_warp_perspective.h warp_kernels.h Makefile
	$(CXX) -c -o $@ $(CFLAGS) $<

%.o: %.cu warp_kernels.h Makefile
	$(NVCC) -c -o $@ $(NVCCFLAGS) $<

$(TARGET_LIB): $(OBJS) Makefile
	$(CXX) -o $@ $(OBJS) $(LIBS)

clean:
	rm -rf $(OBJS) $(TARGET_LIB)

.PHONY: all clean
