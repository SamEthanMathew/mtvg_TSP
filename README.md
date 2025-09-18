# Bounded-Suboptimal Algorithms for the Dubins TSP

This repository is a research and implementation project based on the WAFR 2024 paper “[Bounded-Suboptimal Algorithms for the Dubins TSP](https://arxiv.org/abs/2409.09852).”  
The goal is to reproduce core algorithms, verify results, and extend the ideas for new variants.

---

## Objectives

- **Re-implement** the algorithms described in the paper.
- **Run reproducibility experiments** on benchmark instances.
- **Explore extensions** and further optimize the program.

---

## Setup

Clone the repository (utilizing uv venv for setup):

```
git clone https://github.com/SamEthanMathew/mtvg_TSP.git
cd mtvg_TSP

# Create and sync virtual environment with uv
uv venv .venv
source .venv/bin/activate   # (Linux/Mac)
# Or: .venv\Scripts\activate   # (Windows PowerShell)

# Install dependencies (uv is much faster than pip)
uv pip install -r requirements.txt

# (Optional) Save a lockfile for reproducibility
uv pip freeze > requirements.lock
```


## Milestones

- **Week 1 & 2:** Implement baseline A* and MT-TSP algorithm from Anoop’s prior work. **[DONE]**
- **Week 3:** Implement bounded-suboptimal search algorithms for Dubins TSP.
- **Week 4:** Run initial benchmark tests, verify correctness.
- **Week 5+:** Extend with new variants (moving-target Polyanya, metaheuristics, etc.).



Repo Structure

```
dubins-tsp/
│── src/                 # Core algorithm implementations
│── benchmarks/          # Benchmark instances from WAFR paper
│── results/             # Logs, plots, and experiment results
│── requirements.txt     # Dependencies
│── README.md            # Project overview

```
