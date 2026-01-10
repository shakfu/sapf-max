# Changelog

All notable changes to the sapf~ Max external will be documented in this file.

## [Unreleased]

### Added
- **Audio input support**: New `adc` and `adcn` primitives to read audio from Max inlet
  - `0 adc` - Read from first input channel (returns signal)
  - `1 adc` - Read from second input channel
  - `2 adcn` - Read multiple channels (returns list of signals)
- Example: `0 adc 1000 lpf play` - Low-pass filter input at 1000Hz
- Example: `0 adc dup 0.3 delayl + play` - Add delayed copy to input (echo)
- Thread synchronization mutex for safe message/audio thread interaction

### Changed
- Renamed SAPF library's `post()` function to `sapf_post()` to avoid collision with Max SDK's `post()` function
- Changed message handling from `code` prefix to direct SAPF code input via `anything` method
- Added `list` method handler to support messages starting with numbers (e.g., `5 5 +`)
- Simplified command dispatch - `status`, `help`, `stack`, `clear`, `stop` now handled directly by Max method registration
- Audio inlet now actively used - input is passed to SAPF backend for `adc`/`adcn` access

### Fixed
- Fixed `post()` function name collision between SAPF library and Max SDK that prevented Max console output from working
- Special commands (`status`, `help`, `stack`, `clear`, `stop`) now work correctly
- Added thread safety with mutex protection for compile/execute operations
- Fixed crash in `AdcGen::pull()` when using `adc`/`adcn` primitives - the generator now correctly advances `mOut` to the next list node after `fulfillz()`, and always produces exactly `mBlockSize` samples

### Removed
- Removed `code` message prefix requirement - SAPF code can now be sent directly
- Removed unused thirdparty dependencies (rtaudio, rtmidi, fftw) from embedded SAPF library
- Removed libmanta, tests, and CLI from embedded SAPF library to reduce size
- Removed .git directory from embedded SAPF library

### Internal
- Refactored to use external SAPF library instead of embedded implementation
- SAPF library now located at `source/projects/sapf/`
- Simplified CMakeLists.txt build configuration
- Reduced external bundle size to ~2.5MB
- Reduced SAPF dependency size to ~1.7MB
- Added input buffer management in sapf_tilde for double-to-float conversion
- Added `RegisterMaxAudioPrimitives()` to register Max-specific SAPF functions

## [0.1.0] - Initial Release

### Added
- Basic SAPF language integration with Max/MSP
- Configurable output channels (1-8, default 2)
- Audio generation via `play` command
- Stack inspection and manipulation commands
- Text outlet for stack value output
- Prelude file loading for standard library functions
