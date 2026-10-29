# Small JPEG decoder

It reads a limited, useful subset of JPEG and writes a binary PPM image. The decoder uses only the C++ standard library. There is no image library hidden behind the decoding step.

## Build and run

You need a C++17 compiler and Make.

```sh
make
./build/jpeg_decoder tests/fixtures/color420.jpg build/result.ppm
./build/jpeg_decoder --help
```

`result.ppm` is an ordinary RGB image. PPM stores a short text header followed by three bytes per pixel, which keeps the output code small. Many image viewers support it. The `examples` directory also has an image made by the ray tracer in this collection and decoded here.

Run the tests with Python 3:

```sh
make test
make clean
```

Without Make:

```sh
mkdir -p build
c++ -std=c++17 -O2 -Wall -Wextra -Wpedantic main.cpp -o build/jpeg_decoder
```



## Reading the code

All of the decoding is in `main.cpp`, roughly in the order the file is processed:

1. `Reader` checks byte boundaries and reads marker segments.
2. `Huffman` reconstructs canonical codes from code lengths and symbols.
3. `BitReader` reads individual bits and removes stuffed zero bytes after `FF`.
4. `block` recovers the DC difference and the AC zero runs, then puts the coefficients back into their 8x8 positions.
5. The quantization table rescales the coefficients. A direct inverse discrete cosine transform reconstructs samples.
6. `readScan` places blocks into component planes and resets DC predictors at restart markers.
7. `write` repeats chroma samples to the full resolution, converts YCbCr to RGB, and writes the image.

A minimum coded unit (MCU) is the group of blocks read together. With 4:2:0 sampling it contains four Y blocks, one Cb block and one Cr block. The final MCU can extend past the image edge; the output keeps only the original width and height.

The course connections are bit operations, binary representations, arrays, loops, functions and prefix codes. The cosine transform and colour conversion need some extra maths beyond those courses.



