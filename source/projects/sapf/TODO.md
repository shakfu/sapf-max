# TODO

## Performance Optimization

- [ ] Integrate libxsimd for Linux SIMD optimization
  - The ahihi/sapf fork uses xsimd for cross-platform SIMD support
  - Would provide vectorized math operations on Linux similar to Apple's Accelerate framework
  - Reference: https://github.com/xtensor-stack/xsimd

## Platform Support

- [ ] Investigate Windows build support (libedit dependency issue)

## Build System

- [ ] Consider enabling CI on push/PR (currently manual workflow_dispatch only)
