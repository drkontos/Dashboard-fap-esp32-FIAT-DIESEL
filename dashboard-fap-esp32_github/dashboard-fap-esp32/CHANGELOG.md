# Changelog

## v1.4
- New `Storico` page (Main → Details → History → Options): last 10 regenerations with km since the previous one, duration, max DPF temperature and outcome; average km between regenerations.
- Interrupted regeneration detection: the regeneration in progress is saved every 30 s; at the next start the display reports whether it was resumed by the ECU, finished, or left INTERRUPTED (full-screen warning + sound if the reminder sound is enabled).
- Experimental PIDs: oil degradation (223813) and battery voltage (221955), shown at the bottom of the History page; automatically excluded if the ECU rejects them or returns implausible values.
- Regeneration state is no longer reset by a missing reading right after a reconnection.
- New serial commands `STORICO` and `AZZERA_STORICO`.
- Adapter search controls: STOP button while searching; START / Scegli (manual device list) while paused. The search now runs in 3 s chunks, so STOP and serial commands respond quickly.
- Any serial command sent while the adapter search is running pauses it automatically (bench use with the PC); `START` resumes it.
- Serial `CANCELLA` now asks for confirmation (`SI`) before deleting the log.
- Fault code clearing reworked: UDS `14FFFFFF` first, then OBD `04` on the **functional** address 18DB33F1 (on the Tipo, standard OBD services answer only there, and it is the path a FCA security gateway must leave open for emission DTCs), then `04` on the physical address. Negative responses are parsed correctly (a `7F1433` was read as "no response"), `7F xx 78` (response pending) is ignored, and the result is always verified by re-reading the codes. Every attempt is logged (`DTC_CANC_PROVA`).

## v1.3
- Fault code clearing fixed: on the Tipo the ECU answers the RPM request (010C) only on the functional address; the engine-off check now falls back immediately instead of giving up. If RPM can't be read, clearing proceeds anyway (the ECU rejects it by itself if conditions aren't right).
- Sensor test, step 2: a few throttle blips above ~1800 rpm are enough (4 readings, not necessarily consecutive) instead of holding 3000 rpm.
- Log: GIRI_LETTI / GIRI_NON_LEGGIBILI to diagnose RPM reading.

## v1.2
- New option on the `Opzioni` page: screen-off time without touches (10 s, 30 s, 1 min, 2 min, 5 min or Never = always on).
- With the screen always on, a starting regeneration still switches to the main page.

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
