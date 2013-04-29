#rtl8192c packed Makefile
# (gwl)

rk_cfg := wifi_power.o wifi_power_usb.o wifi_power_ops.o

rkcfg-y += $(rk_cfg)

obj-$(WISKY_WIFI_RTL8188EU) += rkcfg.o
obj-$(WISKY_WIFI_RTL8192C) += rkcfg.o

.PHONY: clean

clean:
	find . -name '*.o*' -exec rm -f {} \; 
