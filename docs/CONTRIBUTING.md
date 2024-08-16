# Contributing Guidelines

Thank you for your interest in contributing to `awviz-ros` 🚀  
We appreciate your effort to help improve this project. Please follow the guidelines below to ensure a smooth collaboration.

## How to Contribute

### 1. Reporting Issues

If you encounter any bugs, have feature requests, or want to ask questions, please [open an issue](https://github.com/ktro2828/awviz-ros/issues).

When reporting an issue, please include:

- A clear and descriptive title.
- A detailed explanation of the problem.
- Steps to reproduce the issue (if applicable).
- Relevant error messages or logs.
- Your environment (OS, ROS version, etc.)

### 2. Submitting Pull Requests

We welcome pull requests (PRs) to address issues, improve documentation, or add new features.  
Here's the process:

1. **Fork the repository** and clone your fork:

   ```bash
   git clone https://github.com/<YOUR-USERNAME>/awviz-ros.git
   cd awviz-ros
   ```

   To ensure changes align with the existing code style, use [pre-commit](https://pre-commit.com/).  
   For the installation, please refer to the official document.

   Before to start making your changes, please run the following command to set up `pre-commit` hooks:

   ```bash
   pre-commit install
   ```

   Now, `pre-commit` will run automatically on `git commit`!

2. **Create a new branch** for your contribution:

   ```bash
   git checkout -b feat/<PACKAGE-NAME>/<YOUR-FEATURE-NAME>
   ```

3. **Make your changes**, ensuring they align with the existing code style. Remember to update or add relevant tests.

4. **Commit your changes** with clear adn descriptive commit messages:

   Note that, we basically follow the [Conventional Commits](https://www.conventionalcommits.org/).

   ```bash
   git add <PATH-TO-CHANGES>
   git commit -sS -m "feat: <YOUR FEATURE>"
   ```

5. **Push your branch** to your fork:

   ```bash
   git push <YOUR-REMOTE> feat/<PACKAGE-NAME>/<YOUR-FEATURE-NAME>
   ```

6. **Submit a pull request** to the main repository. Please describe your changes in detail, linking to any relevant issues.

### 3. Code Style

To maintain a clean and consistent codebase, please follow these guidelines:

- Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
- We recommend to use [clangd](https://clangd.llvm.org/) that features like code completion, navigation (e.g. go to definition, find references), refactoring, and diagnostics.
- Write clear, concise, and well-documented code.
  - For the code documentation, follow the [Doxygen C++ Document Style](https://www.doxygen.nl/manual/index.html).

### 4. Testing

Before submitting a pull request, ensure that all tests pass successfully. You can run the tests using:

```bash
colcon test --packages-select awviz awviz_common awviz_plugin [<PACKAGES>..]
```

If you introduce a new feature or fix a bug, please add appropriate tests to cover the changes.

### 5. Documentation

Well-written documentation is crucial for both users and developers. If your contribution affects the behavior of the system, please ensure that:

- All public functions are documented.
- New features are described in the README or relevant documentation files.

### 6. License

By contributing to `awviz-ros`, you agree that your contributions will be licensed under the project's @ref LICENSE "Apache-2.0".

---

## Get in Touch

If you have any questions, feel free to reach out by opening an issue. We're happy to assist!
