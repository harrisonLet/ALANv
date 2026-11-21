# ALAN Final Report (ECE 4710)

IEEE conference-format LaTeX source for the ALAN final report.

## Build

    latexmk -pdf main.tex

or upload the folder to Overleaf (main document: `main.tex`).

## Layout

| File | Contents |
|---|---|
| `main.tex` | Preamble, title, abstract, keywords, and the list of `\input` files |
| `sections/introduction.tex` | Introduction (from the 3992 proposal) |
| `sections/background.tex` | Terminology, physics of sailing, design options |
| `sections/relatedWork.tex` | Related work (from the 3992 proposal) |
| `sections/overview.tex` | Requirements, specifications, system architecture |
| `sections/tasks/mechanical.tex` | Hull, keel, rudder, wingsail, integration and balance |
| `sections/tasks/electrical.tex` | Power, sensors, radio link, packaging |
| `sections/tasks/software.tex` | Firmware architecture, control, navigation, ground station |
| `sections/testing.tex` | Unit tests, simulator, bench tests, field test, demonstration |
| `sections/impact.tex` | Safety, global, societal, ethical, and regulatory considerations |
| `sections/risks.tex` | Risks: planned mitigation and outcome |
| `sections/management.tex` | Team, schedule, milestones, cost |
| `sections/remaining.tex` | Remaining work |
| `sections/conclusion.tex` | Conclusion |
| `appendices/bom.tex` | Bill of materials |
| `appendices/acknowledgements.tex` | Acknowledgment and AI-use statement |
| `references.bib` | Bibliography |
| `figures/` | Photos, CAD renders, and field-test plots |

## Placeholders

Text the team still has to supply is wrapped in `\tbd{...}` and prints in red.
Search for `\tbd` to find them. Set `\showtbdfalse` in `main.tex` to hide them
when checking layout; remove all of them before submission.

## Rubric checks

* Source files must be ASCII only. Check with
  `LC_ALL=C grep -nP '[^\x00-\x7F]' *.tex sections/*.tex sections/tasks/*.tex appendices/*.tex references.bib`
* Every figure and table needs a `\ref` in the text.
* Author names in `references.bib` must match the publication, with accents
  written as LaTeX escapes (for example `Pr{\"o}ll`).
