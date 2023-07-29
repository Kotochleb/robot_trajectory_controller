# Thesis

Install dependencies
```bash
sudo apt install coinor-libipopt-dev libblas-dev liblapack-dev libsuperlu-dev cmake --install-recommends
# optional for plotting
sudo apt install gnuplot
sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev 
```

Build with:
```bash
mkdir -p build
cd build
cmake -G "Ninja" ..
ninja
```