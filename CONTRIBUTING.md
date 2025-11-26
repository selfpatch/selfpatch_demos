# Contributing to selfpatch_demos

Thanks for your interest in contributing to selfpatch_demos! This guide explains how to report issues, suggest demos, and contribute code.

## How to Report Issues

### Did you find a bug in a demo?

- **Ensure the bug was not already reported** by searching [Issues](https://github.com/selfpatch/selfpatch_demos/issues)
- If you can't find an existing issue, [open a new one](https://github.com/selfpatch/selfpatch_demos/issues/new)
- Include:
  - **Which demo** you were running
  - **Steps to reproduce** - numbered steps to recreate the issue
  - **Expected behavior** - what you expected to happen
  - **Actual behavior** - what actually happened, including error messages
  - **Environment** - ROS 2 distro, OS, ros2_medkit version

### Do you want to suggest a new demo or improvement?

- Check if the idea has already been suggested in [Issues](https://github.com/selfpatch/selfpatch_demos/issues)
- If not, open a new issue describing:
  - **Demo concept** - what would the demo show?
  - **Motivation** - why is this demo valuable?
  - **Prerequisites** - what ROS 2 packages or hardware would be needed?

## How to Contribute Code

### Development Workflow

1. **Fork the repository** and clone your fork locally
2. **Create a branch** from `main` with a descriptive name:
   - `demo/short-description` for new demos
   - `fix/short-description` for bug fixes
   - `docs/short-description` for documentation changes
3. **Make your changes** following the project's structure
4. **Test your changes** locally
5. **Commit your changes** with clear, descriptive commit messages
6. **Push your branch** to your fork
7. **Open a Pull Request** against the `main` branch

### Commit Messages

- Use clear and descriptive commit messages
- Start with a verb in imperative mood (e.g., "Add", "Fix", "Update", "Remove")
- Keep the first line under 72 characters

Examples:
```
Add TurtleBot3 navigation demo

Fix launch file path in turtlebot3_integration

Update README with new prerequisites
```

### Demo Structure

When adding a new demo, follow this structure:

```
demos/your_demo_name/
├── README.md           # Setup instructions and demo description
├── launch/             # ROS 2 launch files
├── config/             # Configuration files
└── src/                # Any custom nodes or scripts (if needed)
```

### Pull Request Checklist

Before submitting your PR, ensure:

- [ ] Demo works with the latest ros2_medkit
- [ ] README clearly explains prerequisites and how to run
- [ ] All dependencies are documented
- [ ] Code follows ROS 2 conventions
- [ ] PR description explains what the demo shows

## Code of Conduct

By contributing to selfpatch_demos, you agree to abide by our [Code of Conduct](CODE_OF_CONDUCT.md).

## License

By contributing to selfpatch_demos, you agree that your contributions will be licensed under the Apache License 2.0.

---

Thank you for helping grow the selfpatch_demos collection! 🤖
