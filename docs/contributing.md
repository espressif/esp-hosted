# Contributing

Thanks for contributing. Please keep each change focused, explain its user or
protocol impact, and include tests or documentation when behaviour changes.

## Before opening a pull request

1. Build the affected host, co-processor, or example configuration.
2. Run the relevant automated tests described in [Testing](testing.md).
3. Run `git diff --check` and update user-facing documentation, migration
   guidance, and the changelog when applicable.
4. Preserve SPDX headers and third-party notices. New files must carry the
   correct SPDX license identifier; match the licensing of the code you change
   unless the maintainer has agreed otherwise.

## Pull requests

Describe the problem, the change, the target host/co-processor and transport,
and validation performed. Do not mix unrelated formatting, generated files, or
large refactors with a functional change.

For protocol, RPC, transport, power-save, or compatibility changes, state the
host/co-processor versions and configurations used for validation. Update
`docs/migration.md` when a change affects upgrade or compatibility behavior.

## Reporting issues

Include the revision, target chips, transport, configuration, logs, and a
minimal reproduction. Remove credentials, addresses, and other sensitive data.
