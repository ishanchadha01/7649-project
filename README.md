# 7649-project - FA-RRT* (A variation on RRTx for partially observable environments)

## [GitHub](https://github.com/ishanchadha01/Field-Aided-RRT)

## Source code
The source code for the search algorithms are in the `src/farrt` folder.
- The current state of the art approach is RRTx. The code can be found at `rrtx2.py`.
- Our new variation uses potential fields to move the orphan points before they are rewired into the tree. It ccan be found at `farrtstar2.py`.

## Instructions to run the code:
1. Setup the python environment
   1. Source the `conda/install.sh` script
   2. OR - use the requirements.txt file provided at the root of the directory
2. Run one of the following commands to see the results of a single algorithm:
   1. `python src/farrt/rrtx2.py`
   2. `python src/farrt/farrtstar2.py`
3. Run `python main.py` to run FA-RRT* and RRTx on the same map and get a comparison.

There is unfortunately not a clean command line interface for the code. To modify the gui settings, manually change `gui=True` to `=False` in the source code.