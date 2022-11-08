#include <cuda.h>
#include <builtin_types.h>


__global__ void test() {
	int deviceCount = 0;
	int cudaDevice = 0;
	char cudaDeviceName[100];

	cuInit(0);
	cuDeviceGetCount(&deviceCount);
	cuDeviceGet(&cudaDevice, 0);
	cuDeviceGetName(cudaDeviceName, 100, cudaDevice);

	std::cout << "device count: " << deviceCount << std::endl;
	std::cout << "device number: " << cudaDevice << std::endl;
	std::cout << "device name: " << cudaDeviceName << std::endl;
}