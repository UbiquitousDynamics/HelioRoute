"""Run the performance evaluation and produce console, JSON, and HTML reports."""

from __future__ import annotations

import datetime as _dt
import argparse
import html
import json
import os

import kpi
from console_utils import safe_print

CAT_ORDER = ["Correctness", "Accuracy", "Latency", "Throughput",
             "Efficiency", "Robustness", "Fidelity"]


def _fmt(v, unit):
    if v is None:
        return "N/A"
    if unit == "bool":
        return "yes" if v >= 1.0 else "no"
    if unit == "%":
        return f"{v:.1f}%"
    if v >= 100:
        return f"{v:.0f}"
    return f"{v:.3f}".rstrip("0").rstrip(".")


def _target_str(op, target, unit):
    sym = "≤" if op == "le" else "≥"
    if unit == "bool":
        return "required"
    return f"{sym} {_fmt(target, unit)}"


def print_dashboard(results, M):
    safe_print("\n" + "=" * 74)
    safe_print(" PERFORMANCE EVALUATION — Solar Car Weather/Route System")
    safe_print(" " + _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    safe_print("=" * 74)
    if "unit_total" in M:
        safe_print(f" Unit tests: {M['unit_total'] - M['unit_failed']}/{M['unit_total']} passed")
    by = {}
    for r in results:
        by.setdefault(r["category"], []).append(r)
    npass = sum(1 for r in results if r["status"] == "PASS")
    for cat in CAT_ORDER:
        if cat not in by:
            continue
        safe_print(f"\n▏{cat}")
        for r in by[cat]:
            mark = {"PASS": "✔", "FAIL": "�’", "ERR": "?"}[r["status"]]
            if r["status"] == "FAIL":
                mark = "�’"
            mark = "✔" if r["status"] == "PASS" else ("✗" if r["status"] == "FAIL" else "?")
            safe_print(f"  {mark} {r['id']}  {r['name']:<38} {_fmt(r['value'], r['unit']):>10} "
                       f"{r['unit']:<6}  target {_target_str(r['op'], r['target'], r['unit'])}")
    safe_print("\n" + "-" * 74)
    safe_print(f" RESULT: {npass}/{len(results)} KPIs on target "
               f"({100 * npass / len(results):.0f}%)")
    safe_print("-" * 74 + "\n")
    return npass


def write_json(results, M, path):
    payload = {"generated": _dt.datetime.now().isoformat(timespec="seconds"),
               "summary": {"kpi_total": len(results),
                           "kpi_pass": sum(1 for r in results if r["status"] == "PASS")},
               "metrics_raw": {k: (None if v is None else v) for k, v in M.items()},
               "kpis": results}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def write_html(results, M, path):
    npass = sum(1 for r in results if r["status"] == "PASS")
    pct = 100 * npass / len(results)
    ring = "#63d18c" if pct >= 90 else ("#f2c14e" if pct >= 70 else "#ef6a5a")
    rows_by = {}
    for r in results:
        rows_by.setdefault(r["category"], []).append(r)

    def bar(r):
        # barra: rapporto valore/target (con verso), clampata a 100%
        v, t, op = r["value"], r["target"], r["op"]
        if v is None or t == 0:
            frac = 0
        elif op == "le":
            frac = max(0.0, min(1.0, 1.0 - min(v, 2 * t) / (2 * t))) if r["unit"] != "bool" else (1.0 if v >= 1 else 0.0)
        else:
            frac = max(0.0, min(1.0, v / t)) if t else 1.0
        col = "#63d18c" if r["status"] == "PASS" else ("#ef6a5a" if r["status"] == "FAIL" else "#8b98a6")
        return f'<div class="bar"><span style="width:{frac*100:.0f}%;background:{col}"></span></div>'

    sections = []
    for cat in CAT_ORDER:
        if cat not in rows_by:
            continue
        trs = []
        for r in rows_by[cat]:
            badge = {"PASS": ("ON TARGET", "#0f2a1a", "#63d18c"),
                     "FAIL": ("OFF TARGET", "#2a1414", "#ef6a5a"),
                     "ERR": ("N/A", "#22262b", "#8b98a6")}[r["status"]]
            trs.append(
                f"<tr><td class='id'>{r['id']}</td><td>{html.escape(r['name'])}</td>"
                f"<td class='num'>{_fmt(r['value'], r['unit'])} <small>{html.escape(r['unit'])}</small></td>"
                f"<td class='num muted'>{_target_str(r['op'], r['target'], r['unit'])}</td>"
                f"<td style='width:180px'>{bar(r)}</td>"
                f"<td><span class='badge' style='color:{badge[2]};background:{badge[1]}'>{badge[0]}</span></td></tr>")
        sections.append(f"<h2>{cat}</h2><table>"
                        f"<tr><th>ID</th><th>KPI</th><th>Value</th><th>Target</th><th></th><th>Result</th></tr>"
                        + "".join(trs) + "</table>")

    gen = _dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    doc = f"""<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Performance report</title>
<style>
:root{{--ink:#e9eef3;--muted:#8b98a6;--line:#22303f;--panel:#141c26;--mono:ui-monospace,Menlo,Consolas,monospace}}
*{{box-sizing:border-box}} body{{margin:0;background:#0b111a;color:var(--ink);font-family:system-ui,Segoe UI,Roboto,sans-serif;padding:26px}}
.wrap{{max-width:980px;margin:0 auto}} h1{{font-size:19px;letter-spacing:.04em}} .sub{{color:var(--muted);font-size:13px}}
.top{{display:flex;gap:22px;align-items:center;background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:18px 22px;margin:16px 0 8px}}
.ring{{width:104px;height:104px;border-radius:50%;background:conic-gradient({ring} {pct*3.6:.0f}deg,#1c2732 0);display:flex;align-items:center;justify-content:center}}
.ring div{{width:80px;height:80px;border-radius:50%;background:var(--panel);display:flex;flex-direction:column;align-items:center;justify-content:center}}
.ring b{{font-size:26px}} .ring small{{color:var(--muted);font-size:11px}}
h2{{font-size:12px;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);margin:22px 0 6px}}
table{{width:100%;border-collapse:collapse;font-size:13.5px;background:var(--panel);border:1px solid var(--line);border-radius:12px;overflow:hidden}}
th,td{{text-align:left;padding:9px 12px;border-bottom:1px solid #1a2632}} th{{color:var(--muted);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.06em}}
tr:last-child td{{border-bottom:none}} .id{{font-family:var(--mono);color:#7f8c99}} .num{{font-family:var(--mono);text-align:right;white-space:nowrap}} .num small{{color:var(--muted)}} .muted{{color:var(--muted)}}
.badge{{font-family:var(--mono);font-size:10.5px;padding:3px 8px;border-radius:20px;font-weight:700;letter-spacing:.03em}}
.bar{{background:#0f1620;border:1px solid #1c2732;border-radius:6px;height:10px;overflow:hidden}} .bar span{{display:block;height:100%}}
.foot{{color:var(--muted);font-size:12px;margin-top:18px;font-family:var(--mono)}}
</style></head><body><div class="wrap">
<h1>Performance report — Solar Car Weather/Route System</h1>
<div class="sub">Generated {gen} · deterministic offline-mock measurements · targets configured in kpi.py</div>
<div class="top"><div class="ring"><div><b>{npass}/{len(results)}</b><small>KPIs on target</small></div></div>
<div><div style="font-size:15px;font-weight:600">{pct:.0f}% of KPIs meet their target</div>
<div class="sub" style="margin-top:4px">Unit tests: {M.get('unit_total','?')} run, {M.get('unit_failed','?')} failed · """ \
        f"""blend improvement over best model: {(_fmt(M.get('blend_improve'), 'x'))}× · """ \
        f"""build 2600 km P95 {(_fmt(M.get('build_p95'),'ms'))} ms</div></div></div>
{''.join(sections)}
<div class="foot">Bar legend: target attainment (green = on target). Boolean KPIs indicate whether the check passed.</div>
</div></body></html>"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(doc)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run offline KPIs and system tests.")
    parser.add_argument("--output-dir", default=None,
                        help="report directory; default: program directory")
    parser.add_argument("--no-write", action="store_true",
                        help="show the dashboard without writing reports")
    args = parser.parse_args(argv)
    results, M = kpi.run()
    # system test end-to-end come ulteriore voce di Correttezza
    try:
        import system_test
        ok, steps = system_test.run_system_test(verbose=False)
        M["system_test"] = 1.0 if ok else 0.0
        M["system_steps"] = f"{sum(1 for _, c, _ in steps if c)}/{len(steps)}"
        results.insert(1, {"id": "STE", "name": "System test end-to-end",
                           "category": "Correctness", "unit": "bool", "op": "ge",
                           "target": 1.0, "value": M["system_test"],
                           "status": "PASS" if ok else "FAIL"})
    except Exception as exc:
        print(f"[eval] system test did not run: {exc}", flush=True)
    print_dashboard(results, M)
    if args.no_write:
        return results, M
    here = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.abspath(args.output_dir or here)
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "perf_report.json")
    html_path = os.path.join(output_dir, "perf_report.html")
    write_json(results, M, json_path)
    write_html(results, M, html_path)
    safe_print(f"Reports written:\n  {html_path}\n  {json_path}")
    return results, M


if __name__ == "__main__":
    main()
