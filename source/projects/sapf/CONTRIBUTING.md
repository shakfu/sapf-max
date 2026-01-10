# Contributing to SAPF

Thank you for your interest in contributing to SAPF (Sound As Pure Form)!

## Getting Started

### Prerequisites

- macOS with Xcode or CMake installed
- Basic knowledge of C++ and audio programming concepts
- Git for version control

### Building the Project

#### Using CMake (Recommended)

```bash
mkdir build && cd build
cmake ..
make -j8
```

For a universal binary (x86_64 + arm64):

```bash
mkdir build && cd build
cmake -DUNIVERSAL=ON ..
make -j8
```

#### Using Xcode

```bash
make  # Uses the Makefile to invoke xcodebuild
```

## Branching Strategy

This project follows a simplified Git Flow model suitable for a small project:

### Main Branch

- **`main`** - The primary branch containing stable, production-ready code
- All releases are tagged on this branch (e.g., `v0.1.22`)
- Protected: should only receive commits via pull requests

### Development Workflow

1. **Create a Feature Branch**
   ```bash
   git checkout -b feature/your-feature-name main
   ```

   Branch naming conventions:
   - `feature/description` - New features
   - `bugfix/description` - Bug fixes
   - `docs/description` - Documentation updates
   - `refactor/description` - Code refactoring

2. **Make Your Changes**
   - Write clear, focused commits
   - Follow existing code style and conventions
   - Add comments for complex logic
   - Update documentation if needed

3. **Test Your Changes**
   - Build the project and verify it compiles without errors
   - Test the functionality you've added or modified
   - Check for memory leaks or crashes

4. **Push Your Branch**
   ```bash
   git push origin feature/your-feature-name
   ```

5. **Create a Pull Request**
   - Go to the GitHub repository
   - Click "New Pull Request"
   - Select your branch
   - Provide a clear description of what your PR does
   - Reference any related issues

## Pull Request Guidelines

### Before Submitting

- [ ] Code compiles without errors or warnings
- [ ] Changes have been tested locally
- [ ] Commits are logical and well-described
- [ ] No unrelated changes are included

### PR Description Should Include

- **What**: Brief description of the changes
- **Why**: Reason for the changes
- **How**: Approach taken (if not obvious)
- **Testing**: How you tested the changes

### Example PR Description

```
## Summary
Fix SEGFAULT when user presses Ctrl-D (EOF)

## Details
The VM was not properly handling EOF condition from the REPL.
Added null pointer check in the readline loop.

## Testing
- Tested with Ctrl-D in REPL - program exits gracefully
- Tested normal operation - works as before
```

## Coding Standards

### Code Style

- Use tabs for indentation (existing project style)
- Follow existing naming conventions:
  - `camelCase` for functions and variables
  - `PascalCase` for class names
  - `UPPER_CASE` for constants
- Keep lines under 120 characters when possible
- Add comments for non-obvious code

### Commit Messages

Write clear, concise commit messages:

```
Good: "Fix SEGFAULT on EOF (^D)"
Good: "Add CMake build support with universal binary option"
Bad: "Fixed stuff"
Bad: "Updates"
```

## Versioning

This project uses [Semantic Versioning](https://semver.org/):

- **MAJOR.MINOR.PATCH** (e.g., 0.1.22)
- **MAJOR**: Incompatible API changes
- **MINOR**: Backward-compatible functionality additions
- **PATCH**: Backward-compatible bug fixes

Version numbers are defined in `src/main.cpp` (`gVersionString`).

## Release Process

Releases are managed by the project maintainer:

1. Merge all PRs for the release into `main`
2. Update version in `src/main.cpp`
3. Commit the version bump
4. Create and push an annotated tag:
   ```bash
   git tag -a v0.1.22 -m "Release v0.1.22"
   git push origin v0.1.22
   ```
5. Create a GitHub release with release notes

## Reporting Issues

### Bug Reports Should Include

- SAPF version (`sapf -h` shows the version)
- macOS version
- Steps to reproduce the issue
- Expected behavior
- Actual behavior
- Any error messages or crash logs

### Feature Requests Should Include

- Clear description of the proposed feature
- Use cases explaining why it would be useful
- Examples of how it might work

## Code of Conduct

- Be respectful and constructive
- Focus on the technical merits of contributions
- Help create a welcoming environment for all contributors

## Questions?

If you have questions about contributing, feel free to:
- Open an issue on GitHub
- Ask in a pull request

## License

By contributing to SAPF, you agree that your contributions will be licensed under the same license as the project (GNU General Public License v3.0 or later).

---

Thank you for contributing to SAPF!
