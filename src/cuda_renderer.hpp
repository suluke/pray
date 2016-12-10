#ifndef PRAY_CUDA_RENDERER_H
#define PRAY_CUDA_RENDERER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"

static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
}
#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

/*Similar to CPU Tracer*/
class CudaRenderer {
  const Scene &scene;
public:
  CudaRenderer(const Scene &scene) : scene(scene) {}
  void render(ImageView &image) const;
};

#endif /*PRAY_CUDA_RENDERER_H*/
