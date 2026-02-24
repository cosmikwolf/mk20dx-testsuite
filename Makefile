SELF_TESTS = watchdog clocks gpio delay timer adc dma pwm i2c usb \
             crc dac flash eeprom lptmr rtc cmp power llwu pwm_advanced ftm_dma

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
