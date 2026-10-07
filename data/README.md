# data/

Captured runs, fitted parameters and generated figures.

## What is here now (worked examples, generated with no hardware)

| file | what it is |
|---|---|
| `step_synthetic.csv` | synthetic step-response data with **known** parameters (K=0.55, tau=0.30, L=0.06, dead-zone=0.15), used to verify the fitter |
| `plant_synthetic.json` | what `sysid_fit.py` recovered from it |
| `fig_sysid.png` | the fit, per step level |
| `fig_sim_compare.png` | with-V2V vs sensor-only, equal headway |
| `fig_design.png` | Bode, margins, closed-loop step |

The synthetic file exists so the whole pipeline can be exercised and checked
before any motor is connected. The fitter recovers tau to 0.2%, the delay to
within one sample, and the dead-zone exactly; the twin then reproduces the true
steady-state speed to within 0.3% at every duty. Delete it once real data
exists, or keep it as a regression check.

## What goes here later

| file | produced by |
|---|---|
| `step.csv` | `log_serial.py` + `firmware/tools/step_response` on the real motor |
| `plant_params.json` | `sysid_fit.py data/step.csv` |
| `gains.json` | `design_control.py` |
| `run_*.csv` | `log_serial.py` during a real run — replayable with `dashboard.py --replay` |

Record a good run early. `dashboard.py --replay` means you can always show real
data even if the hardware misbehaves on review day.
