# CommonChargerStateCurrentChargeStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: @deprecated The charge status of the battery changed.

Header header

# Charger status.
uint8 status_DISCHARGING=0  # The battery is discharging.
uint8 status_CHARGING_SLOW=1  # The battery is charging at a slow rate about 512 mA.
uint8 status_CHARGING_MODERATE=2  # The battery is charging at a moderate rate (&gt; 512 mA) but slower than the fastest rate.
uint8 status_CHARGING_FAST=3  # The battery is charging at a the fastest rate.
uint8 status_BATTERY_FULL=4  # The charger is plugged and the battery is fully charged.
uint8 status
# The current charging phase.
uint8 phase_UNKNOWN=0  # The charge phase is unknown or irrelevant.
uint8 phase_CONSTANT_CURRENT_1=1  # First phase of the charging process. The battery is charging with constant current.
uint8 phase_CONSTANT_CURRENT_2=2  # Second phase of the charging process. The battery is charging with constant current, with a higher voltage than the first phase.
uint8 phase_CONSTANT_VOLTAGE=3  # Last part of the charging process. The battery is charging with a constant voltage.
uint8 phase_CHARGED=4  # The battery is fully charged.
uint8 phase
