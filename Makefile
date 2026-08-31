SELF_TESTS = watchdog clocks clocks_96mhz clocks_120mhz gpio delay timer adc dma pwm i2c usb \
             crc dac flash eeprom lptmr rtc cmp power llwu pwm_advanced pwm_combined \
             ftm_dma spi uart

LOOPBACK_TESTS = gpio_loopback uart_loopback spi_loopback

.PHONY: self-tests loopback-tests all-tests check

self-tests:
	@for test in $(SELF_TESTS); do \
		echo "=== Running $$test ==="; \
		cargo test --test $$test || exit 1; \
	done

loopback-tests:
	@for test in $(LOOPBACK_TESTS); do \
		echo "=== Running $$test ==="; \
		cargo test --test $$test || exit 1; \
	done

all-tests: self-tests loopback-tests

check:
	cargo check --tests
