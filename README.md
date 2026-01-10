# sapf-max

A proof-of-concept project to wrap the [sapf language](https://github.com/lfnoise/sapf) in a Max/MSP external.

## Building

```sh
make
```

## Status

Only works on the macOS platform.

Based on a [my fork](https://github.com/shakfu/sapf), which is included in stripped form in the `source/projects/sapf`, the external correctly interprets sapf code and generates stereo audio via Max's audio thread.

