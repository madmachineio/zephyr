zephyr_library()

zephyr_library_sources(pcie.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_MSI msi.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_SHELL shell.c)
