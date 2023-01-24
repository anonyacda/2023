# sms - Scip MaxCut Solver

## Building

1) make sure SCIP is installed
2) now cmake building should work:

```bash
mkdir build && cd build
cmake ..
make -j 10 # -j 10 makes make use 10 threads
```

If SCIP is not on your path, or you want to use a different version of SCIP as the one on your path:

```bash
mkdir build && cd build
cmake .. -DSCIP_DIR:PATH="{my/path/to/scip}"
make -j 10 # -j 10 makes make use 10 threads
```
