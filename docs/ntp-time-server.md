# Setting Up a Local NTP Time Server

Accurate, consistent time across every device on the hospital network is
important for OpenELIS Global. Result and specimen-collection timestamps,
audit trails, TLS certificate validation, and the ordering of HL7/ASTM/FHIR
messages exchanged with analyzers and the Consolidated Server all depend on
clocks being in sync. If devices are left to drift independently, you can end
up with subtle, hard-to-debug problems such as results that appear "in the
future", certificate errors, or out-of-order audit logs.

The fix is to run a small, local NTP server that every device on the hospital
LAN syncs against. The recommended tool is **chrony** - it is the default NTP
implementation on Ubuntu 24.04, uses only a few MB of RAM and negligible CPU,
and once configured requires essentially no ongoing maintenance.

## 1. Choose a host

Chrony is light enough to run on:

- A small dedicated VM (1 vCPU / 512MB RAM is more than enough), or
- The same Ubuntu 24.04 server that already hosts OpenELIS

If you already have an Ubuntu 24.04 server on site, you do not need a new
machine - just install/configure chrony on it.

## 2. Install chrony

```bash
sudo apt update
sudo apt install -y chrony
```

## 3. Configure it as the hospital's reference server

Edit `/etc/chrony/chrony.conf`:

```bash
sudo nano /etc/chrony/chrony.conf
```

### a) Upstream time sources

Use a pool appropriate to your region, for example:

```
pool 0.asia.pool.ntp.org iburst maxsources 4
pool 1.asia.pool.ntp.org iburst maxsources 1
pool 2.asia.pool.ntp.org iburst maxsources 1
```

(`<region>.pool.ntp.org` works for most countries - replace `asia` with your
country code, e.g. `lk` for Sri Lanka, if a closer pool is available.)

### b) Allow hospital LAN clients to sync against this server

Add an `allow` line for each subnet on the hospital network that should be
permitted to query this server:

```
allow 192.168.0.0/16
allow 10.0.0.0/8
```

Adjust the ranges to match your actual hospital LAN.

### c) Keep serving time even if the internet link is down

Hospital internet connections are often unreliable. Adding a local stratum
fallback lets every device on site continue to sync to this server (and stay
consistent with each other) even when the upstream pool is unreachable:

```
local stratum 10
```

## 4. Enable the service and open the firewall

```bash
sudo systemctl enable --now chrony
sudo ufw allow 123/udp
```

## 5. Verify

```bash
chronyc sources -v
chronyc tracking
```

After a few minutes, `chronyc tracking` should show a system time offset of a
few milliseconds or less.

## 6. Point other devices at this server

- **Linux servers/workstations**: install `chrony`, then in
  `/etc/chrony/chrony.conf` set `server <ntp-server-ip> iburst` and run
  `sudo systemctl restart chrony`.
- **Windows machines**:
  ```cmd
  w32tm /config /manualpeerlist:"<ntp-server-ip>" /syncfromflags:manual /reliable:YES /update
  w32tm /resync
  ```
- **Network devices** (managed switches, routers, UPS units, analyzers with
  NTP support): set the NTP/SNTP server field in the device's admin
  interface to `<ntp-server-ip>`.
- **Docker containers** (including OpenELIS, the database, and the FHIR
  store): containers share the host's kernel clock, so once the Docker host
  is synced via chrony, no extra container configuration is needed.

## Maintenance

Once configured, chrony requires no ongoing attention:

- It automatically re-acquires sync after reboots, network interruptions, or
  VM pauses/snapshots.
- It maintains a drift file (`/var/lib/chrony/drift`) so the local clock
  stays accurate between syncs.
- `chronyc tracking` and `chronyc sources` can be used to spot-check status if
  timestamp-related issues are ever suspected, but no cron jobs, restarts, or
  manual intervention are required in normal operation.
