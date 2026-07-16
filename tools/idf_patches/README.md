# ESP-IDF patches

ESP-Hosted applies a small patch on top of the pinned ESP-IDF (currently
`v5.5.4`; see `IDF_DEFAULT_REF` in `tools/eh.py`) when required.

The patch is applied automatically by `eh.py install` or can be applied
manually using `eh.py patch-idf`.

## When is the patch needed?

### No patch required

The SDIO fix is already present in the following ESP-IDF releases:

- `v5.3` – `v5.3.4`
- `v5.4`, `v5.4.2`, `v5.4.3`
- `v5.5`, `v5.5.1`, `v5.5.2`
- `master`

If you are using one of these versions (or any later release containing the
fix), no patch is required.

### Patch required

If your ESP-IDF revision does not include the fix, the patch must be applied.

## Manual patch

In `components/esp_driver_sdio/src/sdio_slave.c`, replace:

```
SDIO_SLAVE_CHECK(len > 0 && len <= 4092, "length out of range: (0, 4092]", ESP_ERR_INVALID_ARG);
```
with:
```
SDIO_SLAVE_CHECK(len > 0, "len <= 0", ESP_ERR_INVALID_ARG);
```

## Automatic patching

You normally do **not** need to apply this patch manually.

If you reuse an existing ESP-IDF checkout, ESP-Hosted automatically checks
whether the fix is already present.

You can point ESP-Hosted to an existing ESP-IDF using:

```bash
eh.py set-idf-path <path>
```

`eh.py` inspects the selected `$IDF_PATH` and determines whether the SDIO patch
is required. If needed, it prompts you to run:

```bash
eh.py patch-idf
```

which automatically updates the required line.

For an emulator or hardware test run, `eh.py test emu --auto-apply-idf-patches` (or `eh.py test hw --auto-apply-idf-patches`) applies the same change in place; without it, the test reports the required `patch-idf` command and stops.
