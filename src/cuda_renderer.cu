/**
Raytracer that takes pixel size and scene as input
*/

#include<cuda_renderer.hpp>

__device__ static TriangleIndex intersectTriangle(const Scene &scene, const Ray &ray, float *out_distance)
{
        TriangleIndex intersected_triangle = TriangleIndex_Invalid;
        float minimum_distance = std::numeric_limits<float>::max();

        for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(scene.triangles.size()); ++triangle_index)
        {
                const Triangle &triangle = scene.triangles[triangle_index];

                float distance;
                if(intersectRayTriangle(ray.origin, ray.direction, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2], &distance))
                {
                        if(distance < minimum_distance)
                        {
                                intersected_triangle = triangle_index;
                                minimum_distance = distance;
                        }
                }
        }

        *out_distance = minimum_distance;
        return intersected_triangle;
}

__device__ static Color trace(const Scene &scene, const Ray &ray)
{
        float intersection_distance;
        const TriangleIndex intersected_triangle = intersectTriangle(scene, ray, &intersection_distance);

        if(intersected_triangle == TriangleIndex_Invalid) return scene.background_color;

        const Vector3 N = scene.triangles[intersected_triangle].calculateNormal();

        // optimization: back faces are never lit
        if(ray.direction.dot(N) >= 0.f) return Color(0.f, 0.f, 0.f);

        const Vector3 P = ray.origin + ray.direction * intersection_distance;

        const MaterialIndex material_index = scene.triangles[intersected_triangle].material_index;
        ASSERT(material_index != MaterialIndex_Invalid);

        Color result_color(0.f, 0.f, 0.f);

        for(auto &light : scene.lights)
        {
                const Vector3 light_vector = light.position - P;
                const float light_distance = light_vector.length();
                const Vector3 L = light_vector / light_distance;

                const Ray shadow_ray(P + L * 0.001f, L);
                if(intersectTriangle(scene, shadow_ray, &intersection_distance) == TriangleIndex_Invalid || intersection_distance > light_distance)
                {
                        // don't use intersection_distance here, use light_distance instead!
                        const Color shading_color = scene.materials[material_index].color * light.color * (std::max(L.dot(N), 0.f) / (light_distance * light_distance));
                        result_color += shading_color;
                }
        }

        return result_color;
}

void CudaRenderer::render(ImageView &image) const
{

    //TODO: init Datastructure on card




        Vector3 left, right, bottom, top;
        // assuming fov is in x direction, otherwise invert this
        const float aspect = (float)image.resolution.h / image.resolution.w;
        scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

        float max_x = (float) image.resolution.w;
        float max_y = (float) image.img.resolution.h;

        //Cuda Variables
        float *cuda_max_x;
        float *cuda_max_y;
        Vector3 *cuda_left;
        Vector3 *cuda_top;
        dim_t *cuda_height;
        dim_t *cuda_width;

        HANDLE_ERROR( cudaMalloc(&cuda_max_x, sizeof(float)) );
        HANDLE_ERROR( cudaMalloc(&cuda_max_y, sizeof(float)) );
        HANDLE_ERROR( cudaMalloc(&cuda_left, sizeof(Vector3)) );
        HANDLE_ERROR( cudaMalloc(&cuda_top, sizeof(Vector3)) );
        HANDLE_ERROR( cudaMalloc(&cuda_height, sizeof(dim_t)) );
        HANDLE_ERROR( cudaMalloc(&cuda_width, sizeof(dim_t)) );

        raytracer<<<1,8>>>(cuda_max_x,cuda_max_y,cuda_left,cuda_height,cuda_height,cuda_width);


        #ifdef _MSC_VER // msvc does not support uint32_t as index variable in for loop
                #pragma omp parallel for
                for(intmax_t y = 0; y < image.resolution.h; ++y)
        #else
                #pragma omp parallel for
                for(uint32_t y = 0; y < image.resolution.h; ++y)
        #endif
                {
                        const float i_y = 1.f - (2 * image.getGlobalY(y) + 1) / max_y;
                        for(uint32_t x = 0; x < image.resolution.w; ++x)
                        {
                                const float i_x = 1.f - (2 * x + 1) / max_x;

                                Ray ray(scene.camera.position, (left * i_x + top * i_y + scene.camera.direction).normalize());

                                Color c = trace(scene, ray);
                                image.setPixel(x, y, c);
                        }
                }
        }


        bool initDatastructures () {
            //Init the datastructures for cuda


            //Init Scene
        }

        __global__ void raytracer (Scene *scene, ImageView* image, float* max_x, float* max_y,
                                   Vector3 *left, Vector3 *top, dim_t *height, dim_t *width) {
            //Calculate my position to write
            unsigned int id = threadIdx.x + blockDim.x * blockIdx.x;
            unsigned int i_x = id /

        }

        __device__ shootRayPacket () {

        }
