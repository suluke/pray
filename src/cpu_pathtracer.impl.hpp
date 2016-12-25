template <class ray_t, class accel_t>
void CpuPathTracer<ray_t, accel_t>::preprocess() {

}

template <class ray_t, class accel_t>
void CpuPathTracer<ray_t, accel_t>::render(ImageView &image) const {

}

template <class ray_t, class accel_t>
typename ray_t::color_t CpuPathTracer<ray_t, accel_t>::trace(const Scene &scene, const ray_t &ray) const {
  return {Color()};
}
