zephyr_include_directories(.)

zephyr_sources(
  check_regs.c
  soc_irq.S
  soc_common_irq.c
  vector.S
  )
