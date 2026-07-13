#!/usr/bin/env python3
"""
OpenOBD CAN-Analyse-Kit
=======================
Verdichtet große CAN-Rohlogs (SavvyCAN-CSV, wie OpenOBD sie schreibt) zu
kompakten Zusammenfassungen, die man einer KI zum Deuten geben kann.

WARUM: Ein 30-MB-Log hat ~1 Mio. Frames. Das kann keine KI im Chat "durchrechnen".
Dieses Skript rechnet lokal (kostenlos) und gibt dir ~1 Seite Text, die in jeden
KI-Prompt passt. -> Python rechnet, die KI interpretiert.

ABHÄNGIGKEITEN: python3, pandas, numpy  (schon vorhanden auf dem Mac)

BENUTZUNG:
  python3 analyze.py ../results/raw_s030.csv
      -> ID-Inventar + Bit-Flip-Analyse + UDS-Übersicht (nach stdout)

  python3 analyze.py ../results/raw_s030.csv --out bericht_s030.md
      -> dasselbe zusätzlich als Markdown-Datei (die gibst du der KI)

  python3 analyze.py ../results/raw_s030.csv --id 18DAF101
      -> nur diese eine CAN-ID im Detail (jedes Byte, Wertebereich, Bit-Toggle)

  python3 analyze.py ../results/raw_s030.csv --corr ../results/s030.csv --signal Speed_kmh
      -> Korrelation: welches Byte/16-Bit-Wort korreliert mit einem bekannten
         Signal (z.B. Speed_kmh, RPM) aus dem dekodierten Log? Das ist die
         "Reference-Signal"-Methode, um unbekannte Signale zu finden.
"""
import sys, argparse
import numpy as np
import pandas as pd

# Raw-Zeitstempel von OpenOBD sind Mikrosekunden (esp_timer_get_time()).
TS_TO_SEC = 1e-6

DBYTES = ["D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8"]


def load_raw(path):
    """Lädt ein SavvyCAN-Roh-CSV und normalisiert die Spalten."""
    df = pd.read_csv(path, dtype=str).fillna("")
    df.columns = [c.strip() for c in df.columns]
    if "Time Stamp" not in df.columns or "ID" not in df.columns:
        sys.exit(f"FEHLER: {path} sieht nicht wie ein SavvyCAN-Roh-CSV aus "
                 f"(Spalten: {list(df.columns)})")
    df["ts"] = pd.to_numeric(df["Time Stamp"], errors="coerce")
    df["sec"] = df["ts"] * TS_TO_SEC
    df["ID"] = df["ID"].str.upper().str.strip()
    df["LEN"] = pd.to_numeric(df["LEN"], errors="coerce").fillna(0).astype(int)
    df["Dir"] = df.get("Dir", "").str.strip()
    # Datenbytes als 0..255 int (leer -> -1)
    for b in DBYTES:
        if b in df.columns:
            df[b] = df[b].apply(lambda x: int(x, 16) if x.strip() else -1)
        else:
            df[b] = -1
    return df.dropna(subset=["ts"]).reset_index(drop=True)


def id_inventory(df):
    """Tabelle: pro CAN-ID Anzahl, Rate, Richtung, DLC, verschiedene Payloads."""
    dur = max(df["sec"].max() - df["sec"].min(), 1e-9)
    rows = []
    for cid, g in df.groupby("ID"):
        payloads = g[DBYTES].astype(int)
        # eindeutige Payloads (nur gültige Bytes gemäß LEN)
        keys = payloads.apply(lambda r: tuple(r[:g["LEN"].iloc[0]]) if g["LEN"].iloc[0] > 0
                              else tuple(r), axis=1)
        rows.append({
            "ID": cid,
            "Dir": "/".join(sorted(g["Dir"].unique())),
            "Frames": len(g),
            "Rate_Hz": round(len(g) / dur, 2),
            "DLC": int(g["LEN"].mode().iloc[0]) if len(g) else 0,
            "Payloads": keys.nunique(),
        })
    inv = pd.DataFrame(rows).sort_values("Frames", ascending=False).reset_index(drop=True)
    return inv, dur


def bitflip(df, cid):
    """Pro Byte einer ID: Wertebereich + welche Bits sich je ändern.
    Hilft, Zähler/Checksummen/konstante Bytes/echte Signale zu unterscheiden."""
    g = df[df["ID"] == cid]
    if g.empty:
        return f"(keine Frames für ID {cid})"
    dlc = int(g["LEN"].mode().iloc[0])
    dlc = dlc if dlc > 0 else 8
    lines = [f"ID {cid}  |  {len(g)} Frames  |  DLC {dlc}"]
    lines.append("Byte  min  max   toggle-Bits (7..0)   Deutung-Hinweis")
    for i in range(dlc):
        col = g[DBYTES[i]]
        col = col[col >= 0]
        if col.empty:
            continue
        vals = col.to_numpy().astype(int)
        mn, mx = int(vals.min()), int(vals.max())
        # Welche Bits wechseln über den Verlauf mindestens einmal?
        orbits = np.bitwise_or.reduce(vals)
        andbits = np.bitwise_and.reduce(vals)
        toggling = orbits & ~andbits  # Bits, die mal 0 mal 1 sind
        bmap = "".join("X" if (toggling >> b) & 1 else
                       ("1" if (andbits >> b) & 1 else "0") for b in range(7, -1, -1))
        # grobe Deutung
        span = mx - mn
        if span == 0:
            hint = "konstant"
        elif bin(toggling).count("1") >= 7 and len(np.unique(vals)) > len(vals) * 0.5:
            hint = "evtl. Zähler/Checksumme (fast alle Bits wechseln)"
        elif span <= 4:
            hint = "Status/kleiner Bereich"
        else:
            hint = "Signal-Kandidat"
        lines.append(f" D{i+1}   {mn:3d}  {mx:3d}   {bmap}   {hint}")
    lines.append("Legende: 0/1 = Bit immer 0/1 (statisch), X = wechselt")
    return "\n".join(lines)


def uds_overview(df):
    """Fasst UDS-Diagnoseverkehr zusammen (29-Bit 18DAxxF1 / 18DAF1xx).
    Erkennt Single-Frame-Requests/Responses, Services, positive/negative Antworten."""
    NRC = {0x10: "generalReject", 0x11: "serviceNotSupported",
           0x12: "subFunctionNotSupported", 0x13: "wrongLength/Format",
           0x22: "conditionsNotCorrect", 0x31: "requestOutOfRange",
           0x33: "securityAccessDenied", 0x35: "invalidKey",
           0x78: "responsePending", 0x7E: "svcNotSuppInSession",
           0x7F: "svcNotSuppInSession"}
    SVC = {0x10: "DiagSession", 0x22: "ReadDataByID", 0x3E: "TesterPresent",
           0x27: "SecurityAccess", 0x2E: "WriteDataByID", 0x31: "RoutineControl",
           0x19: "ReadDTC", 0x01: "OBD-Mode01", 0x09: "OBD-Mode09"}
    mask = df["ID"].str.startswith("18DA")
    u = df[mask]
    if u.empty:
        return "(kein 29-Bit-UDS/OBD-Diagnoseverkehr 18DA.. gefunden)"
    lines = [f"UDS/OBD-Diagnoseframes (18DA..): {len(u)}"]
    seen = {}
    for _, r in u.iterrows():
        b = [r[d] for d in DBYTES]
        pci = b[0] & 0xF0 if b[0] >= 0 else -1
        cid = r["ID"]
        if pci == 0x00:  # Single Frame
            svc = b[1]
            if svc == 0x7F:
                key = (cid, "NEG", b[2], b[3])
                desc = f"{cid}  NEG  svc {b[2]:02X} {SVC.get(b[2],'?')}  NRC {b[3]:02X} {NRC.get(b[3],'?')}"
            elif svc >= 0x40:  # positive Antwort (Service+0x40)
                key = (cid, "POS", svc, b[2])
                desc = f"{cid}  POS  svc {svc-0x40:02X} {SVC.get(svc-0x40,'?')}  data {b[2]:02X} {b[3]:02X} {b[4]:02X}..."
            else:  # Request
                key = (cid, "REQ", svc, b[2])
                desc = f"{cid}  REQ  svc {svc:02X} {SVC.get(svc,'?')}  {b[2]:02X} {b[3]:02X}"
            seen[key] = seen.get(key, 0)
            seen[key] += 1
            if seen[key] == 1:
                lines.append("  " + desc)
        elif pci == 0x10:  # First Frame (Multiframe)
            k = (cid, "FF")
            if k not in seen:
                seen[k] = 1
                lines.append(f"  {cid}  MULTIFRAME (First Frame) - braucht Flow Control")
    # Häufigkeiten anhängen
    lines.append("--- Häufigkeiten ---")
    for k, n in sorted(seen.items(), key=lambda x: -x[1])[:20]:
        lines.append(f"  {n:6d}x  {k}")
    return "\n".join(lines)


def correlate(df, decoded_path, signal):
    """Reference-Signal-Methode: korreliert jedes Byte & 16-Bit-Wort jeder ID
    mit einem bekannten Signal aus dem dekodierten Log. Top-Treffer = Kandidaten."""
    dec = pd.read_csv(decoded_path)
    if signal not in dec.columns:
        sys.exit(f"Signal '{signal}' nicht in {decoded_path}. Verfügbar:\n{list(dec.columns)}")
    dec = dec.dropna(subset=["Timestamp_ms", signal])
    dec_t = dec["Timestamp_ms"].to_numpy() / 1000.0  # -> Sekunden
    dec_v = dec[signal].to_numpy(dtype=float)
    if len(dec_v) < 5 or np.std(dec_v) == 0:
        sys.exit(f"Referenzsignal '{signal}' hat zu wenig Variation zum Korrelieren.")

    results = []
    for cid, g in df.groupby("ID"):
        g = g.sort_values("sec")
        if len(g) < 5:
            continue
        t = g["sec"].to_numpy()
        dlc = int(g["LEN"].mode().iloc[0]) or 8
        for i in range(dlc):
            raw_v = g[DBYTES[i]].to_numpy(dtype=float)
            if (raw_v < 0).any() or np.std(raw_v) == 0:
                continue
            # dekodiertes Signal auf die Frame-Zeitpunkte interpolieren
            ref = np.interp(t, dec_t, dec_v)
            if np.std(ref) == 0:
                continue
            c = np.corrcoef(raw_v, ref)[0, 1]
            if not np.isnan(c):
                results.append((abs(c), c, cid, f"D{i+1} (8bit)"))
        # 16-Bit big-endian Worte
        for i in range(dlc - 1):
            hi = g[DBYTES[i]].to_numpy(dtype=float)
            lo = g[DBYTES[i + 1]].to_numpy(dtype=float)
            if (hi < 0).any() or (lo < 0).any():
                continue
            raw_v = hi * 256 + lo
            if np.std(raw_v) == 0:
                continue
            ref = np.interp(t, dec_t, dec_v)
            c = np.corrcoef(raw_v, ref)[0, 1]
            if not np.isnan(c):
                results.append((abs(c), c, cid, f"D{i+1}:D{i+2} (16bit BE)"))
    results.sort(reverse=True)
    lines = [f"Korrelation gegen Referenzsignal '{signal}' (Top 25):",
             "|corr|   corr    ID          Byte(s)"]
    for a, c, cid, pos in results[:25]:
        lines.append(f"{a:5.2f}  {c:+5.2f}   {cid:<10}  {pos}")
    lines.append("Hinweis: |corr| nahe 1.0 = starker Kandidat für dieses Signal.")
    lines.append("ACHTUNG am OBD-Port dieses Golf: nur Diagnose-Antworten (18DAF1xx) fließen,")
    lines.append("die sind schon OBD-dekodiert -> passive Korrelation findet hier wenig Neues.")
    lines.append("Voll nützlich wird das bei UDS-DID-Antworten oder an einem direkten CAN-Abgriff.")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="OpenOBD CAN-Analyse")
    ap.add_argument("rawcsv", help="Pfad zu raw_sNNN.csv")
    ap.add_argument("--id", help="nur diese CAN-ID im Detail (hex, z.B. 18DAF101)")
    ap.add_argument("--corr", help="dekodiertes sNNN.csv für Korrelation")
    ap.add_argument("--signal", default="Speed_kmh", help="Referenzsignal für --corr")
    ap.add_argument("--out", help="Bericht zusätzlich als Markdown-Datei speichern")
    args = ap.parse_args()

    df = load_raw(args.rawcsv)
    out = []
    out.append(f"# CAN-Analyse: {args.rawcsv}")
    out.append(f"Frames: {len(df)}  |  Dauer: {df['sec'].max()-df['sec'].min():.1f} s  "
               f"|  eindeutige IDs: {df['ID'].nunique()}")

    if args.corr:
        out.append("\n## Korrelation\n```\n" + correlate(df, args.corr, args.signal) + "\n```")
    elif args.id:
        out.append("\n## Bit-Analyse\n```\n" + bitflip(df, args.id.upper()) + "\n```")
    else:
        inv, dur = id_inventory(df)
        out.append("\n## ID-Inventar\n```\n" + inv.to_string(index=False) + "\n```")
        out.append("\n## UDS/Diagnose-Übersicht\n```\n" + uds_overview(df) + "\n```")
        # Bit-Analyse für die 3 häufigsten Daten-IDs (nur Rx / echte Antworten)
        top = inv[inv["Frames"] > 20]["ID"].head(4).tolist()
        for cid in top:
            out.append(f"\n## Bit-Analyse {cid}\n```\n" + bitflip(df, cid) + "\n```")

    report = "\n".join(out)
    print(report)
    if args.out:
        with open(args.out, "w") as f:
            f.write(report)
        print(f"\n[gespeichert: {args.out}]", file=sys.stderr)


if __name__ == "__main__":
    main()
