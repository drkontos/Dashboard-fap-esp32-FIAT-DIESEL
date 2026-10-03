# Changelog

## v1.1
- Adapter search lasts up to 15 s but stops as soon as the adapter is seen (`DURATA_RICERCA_S`).
- The configured adapter (MAC or name) is always accepted, even if another device was picked from the list.
- A device that connects but doesn't answer OBD commands is automatically forgotten.
- The device list appears only on the 3rd failed attempt (then every 5) and stays for 30 s.
- On-screen hints when the adapter isn't found (ignition on? phone connected to the adapter?).
- Screen stays on while searching for the adapter.
- New option on the `Opzioni` page: how long the screen stays on after connecting (10 s – 5 min, default 1 min).
- New serial command `DIMENTICA`.
- More detailed event log (adapter seen / not seen, search time).

## v1.0
- First public release.
