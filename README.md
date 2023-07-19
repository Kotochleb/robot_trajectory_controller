# Thesis

Install dependencies
```bash
sudo apt install coinor-libipopt-dev libblas-dev liblapack-dev cmake --install-recommends
# optional for plotting
sudo apt install gnuplot
```

Build with:
```bash
mkdir -p build
cd build
cmake ..
make
```