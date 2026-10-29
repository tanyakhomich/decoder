#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using Bytes = std::vector<unsigned char>;

const std::array<int, 64> zigzag = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Reader {
    const Bytes& data;
    std::size_t pos, end;

    int byte() {
        require(pos < end, "truncated JPEG");
        return data[pos++];
    }

    int word() {
        int high = byte();
        return high * 256 + byte();
    }

    Reader segment() {
        int length = word();
        require(length >= 2 && std::size_t(length - 2) <= end - pos,
                "invalid or truncated segment");
        std::size_t start = pos;
        pos += length - 2;
        return {data, start, pos};
    }

    int marker() {
        require(byte() == 0xff, "expected a JPEG marker");
        int value;
        do { value = byte(); } while (value == 0xff);
        require(value != 0, "stuffed byte outside image data");
        return value;
    }

    void finished() const {
        require(pos == end, "unexpected bytes in segment");
    }
};

struct BitReader {
    Reader& input;
    int current = 0, left = 0;

    int bit() {
        if (left == 0) {
            current = input.byte();
            if (current == 0xff)
                require(input.byte() == 0, "marker inside an unfinished block");
            left = 8;
        }
        --left;
        return (current >> left) & 1;
    }

    int bits(int count) {
        int result = 0;
        for (int i = 0; i < count; ++i) result = result * 2 + bit();
        return result;
    }

    void align() {
        int mask = (1 << left) - 1;
        require((current & mask) == mask, "invalid scan padding");
        left = 0;
    }
};

struct Huffman {
    bool present = false;
    std::array<int, 17> count{}, firstCode{}, firstSymbol{};
    Bytes symbols;

    void read(Reader& input, bool ac) {
        int total = 0;
        for (int length = 1; length <= 16; ++length) {
            count[length] = input.byte();
            total += count[length];
        }
        require(total > 0 && total <= 256, "invalid Huffman table size");
        symbols.clear();
        for (int i = 0; i < total; ++i) {
            int symbol = input.byte();
            if (ac) {
                int size = symbol & 15;
                require(size <= 10 && (size != 0 || symbol == 0 || symbol == 0xf0),
                        "invalid baseline AC symbol");
            } else {
                require(symbol <= 11, "invalid baseline DC symbol");
            }
            symbols.push_back(static_cast<unsigned char>(symbol));
        }
        int code = 0, offset = 0;
        for (int length = 1; length <= 16; ++length) {
            firstCode[length] = code;
            firstSymbol[length] = offset;
            require(code + count[length] < (1 << length),
                    "oversubscribed Huffman table or forbidden all-one code");
            code = (code + count[length]) * 2;
            offset += count[length];
        }
        present = true;
    }

    int decode(BitReader& input) const {
        int code = 0;
        for (int length = 1; length <= 16; ++length) {
            code = code * 2 + input.bit();
            int index = code - firstCode[length];
            if (index >= 0 && index < count[length])
                return symbols[firstSymbol[length] + index];
        }
        throw std::runtime_error("invalid Huffman code");
    }
};

struct Component {
    int id, h, v, quant;
    int dcTable = 0, acTable = 0, previousDC = 0;
    int stride = 0;
    Bytes samples;
};

unsigned char sample(double value) {
    value = std::max(0.0, std::min(255.0, std::round(value)));
    return static_cast<unsigned char>(value);
}

class Decoder {
    Reader input;
    std::array<std::array<int, 64>, 4> quant{};
    std::array<bool, 4> quantPresent{};
    std::array<Huffman, 2> dcTables{}, acTables{};
    std::vector<Component> components;
    std::array<std::array<double, 8>, 8> cosine{};
    int width = 0, height = 0, maxH = 1, maxV = 1;
    int restartInterval = 0, adobeTransform = -1;
    bool decoded = false;

    void readQuant(Reader& segment) {
        require(segment.pos < segment.end, "empty quantization segment");
        while (segment.pos < segment.end) {
            int info = segment.byte();
            int id = info & 15;
            require((info >> 4) == 0 && id < 4,
                    "only 8-bit quantization tables 0..3 are supported");
            for (int i = 0; i < 64; ++i) {
                int value = segment.byte();
                require(value != 0, "zero quantization value");
                quant[id][zigzag[i]] = value;
            }
            quantPresent[id] = true;
        }
    }

    void readHuffman(Reader& segment) {
        require(segment.pos < segment.end, "empty Huffman segment");
        while (segment.pos < segment.end) {
            int info = segment.byte();
            int kind = info >> 4, id = info & 15;
            require(kind <= 1 && id < 2,
                    "baseline Huffman table class/id is out of range");
            if (kind == 0) dcTables[id].read(segment, false);
            else acTables[id].read(segment, true);
        }
    }

    void readFrame(Reader& segment) {
        require(components.empty(), "multiple frames are not supported");
        require(segment.byte() == 8, "only 8-bit JPEG is supported");
        height = segment.word();
        width = segment.word();
        require(width > 0 && height > 0, "empty image or deferred height");
        require(width <= 4096 && height <= 4096 &&
                    std::uint64_t(width) * height <= 4194304,
                "image limit: 4096 per side and 4,194,304 pixels");
        int count = segment.byte();
        require(count == 1 || count == 3, "only grayscale and YCbCr are supported");
        for (int i = 0; i < count; ++i) {
            int id = segment.byte(), sampling = segment.byte(), table = segment.byte();
            int h = sampling >> 4, v = sampling & 15;
            require(h >= 1 && h <= 2 && v >= 1 && v <= 2 && table < 4,
                    "unsupported sampling factors or quantization table");
            components.push_back({id, h, v, table, 0, 0, 0, 0, {}});
        }
        segment.finished();
        if (count == 1) {
            require(components[0].h == 1 && components[0].v == 1,
                    "grayscale sampling must be 1x1");
        } else {
            for (int i = 0; i < 3; ++i)
                require(components[i].id == i + 1,
                        "YCbCr component ids must be 1, 2, 3 in that order");
            require(components[1].h == 1 && components[1].v == 1 &&
                        components[2].h == 1 && components[2].v == 1 &&
                        !(components[0].h == 1 && components[0].v == 2),
                    "only 4:4:4, 4:2:2 and 4:2:0 sampling is supported");
            maxH = components[0].h;
            maxV = components[0].v;
        }
    }

    int signedValue(BitReader& bits, int size) {
        if (size == 0) return 0;
        int value = bits.bits(size);
        if (value < (1 << (size - 1))) value -= (1 << size) - 1;
        return value;
    }

    void block(BitReader& bits, Component& component, int x, int y) {
        std::array<int, 64> coefficients{};
        int size = dcTables[component.dcTable].decode(bits);
        component.previousDC += signedValue(bits, size);
        require(component.previousDC >= -2048 && component.previousDC <= 2047,
                "DC coefficient outside the 8-bit baseline range");
        coefficients[0] = component.previousDC;
        int k = 1;
        while (k < 64) {
            int symbol = acTables[component.acTable].decode(bits);
            if (symbol == 0) break;
            int run = symbol >> 4;
            size = symbol & 15;
            if (symbol == 0xf0) {
                k += 16;
                require(k <= 64, "AC zero run exceeds the block");
                continue;
            }
            k += run;
            require(k < 64, "AC coefficient exceeds the block");
            coefficients[zigzag[k++]] = signedValue(bits, size);
        }
        for (int i = 0; i < 64; ++i)
            coefficients[i] *= quant[component.quant][i];

        // Direct IDCT: slow, but the formula is visible here.
        for (int row = 0; row < 8; ++row) {
            for (int col = 0; col < 8; ++col) {
                double sum = 0;
                for (int v = 0; v < 8; ++v)
                    for (int u = 0; u < 8; ++u)
                        sum += coefficients[v * 8 + u] * cosine[col][u] * cosine[row][v];
                component.samples[std::size_t(y + row) * component.stride + x + col] =
                    sample(sum / 4 + 128);
            }
        }
    }

    void readScan(Reader& segment) {
        require(!components.empty(), "scan before frame header");
        require(!decoded, "multiple scans are not supported");
        require(adobeTransform == -1 ||
                    adobeTransform == (components.size() == 3 ? 1 : 0),
                "Adobe RGB, CMYK and YCCK JPEGs are not supported");
        int count = segment.byte();
        require(count == int(components.size()), "separate component scans are not supported");
        std::vector<int> order;
        for (int i = 0; i < count; ++i) {
            int id = segment.byte(), tables = segment.byte();
            int index = -1;
            for (int j = 0; j < count; ++j)
                if (components[j].id == id) index = j;
            require(index >= 0 && std::find(order.begin(), order.end(), index) == order.end(),
                    "unknown or repeated scan component");
            Component& component = components[index];
            component.dcTable = tables >> 4;
            component.acTable = tables & 15;
            require(component.dcTable < 2 && component.acTable < 2,
                    "Huffman table index is out of range");
            require(quantPresent[component.quant] && dcTables[component.dcTable].present &&
                        acTables[component.acTable].present, "missing decoding table");
            order.push_back(index);
        }
        int start = segment.byte(), end = segment.byte(), approximation = segment.byte();
        require(start == 0 && end == 63 && approximation == 0,
                "only complete sequential scans are supported");
        segment.finished();

        int columns = (width + 8 * maxH - 1) / (8 * maxH);
        int rows = (height + 8 * maxV - 1) / (8 * maxV);
        for (Component& component : components) {
            component.stride = columns * component.h * 8;
            component.samples.resize(std::size_t(component.stride) * rows * component.v * 8);
        }
        BitReader bits{input};
        int mcu = 0, restart = 0;
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < columns; ++col) {
                if (restartInterval && mcu && mcu % restartInterval == 0) {
                    bits.align();
                    require(input.marker() == 0xd0 + restart, "missing or wrong restart marker");
                    restart = (restart + 1) % 8;
                    for (Component& component : components) component.previousDC = 0;
                }
                for (int index : order) {
                    Component& component = components[index];
                    for (int v = 0; v < component.v; ++v)
                        for (int h = 0; h < component.h; ++h)
                            block(bits, component, (col * component.h + h) * 8,
                                  (row * component.v + v) * 8);
                }
                ++mcu;
            }
        }
        bits.align();
        decoded = true;
    }

    void readApp(int marker, Reader& segment) {
        if (marker != 0xee || segment.end - segment.pos < 5) return;
        std::string signature;
        for (int i = 0; i < 5; ++i) signature += char(segment.byte());
        if (signature != "Adobe") return;
        require(!decoded, "Adobe colour marker after scan is not supported");
        require(segment.end - segment.pos >= 7, "truncated Adobe marker");
        segment.word();
        segment.word();
        segment.word();
        int transform = segment.byte();
        require(adobeTransform == -1 || adobeTransform == transform,
                "conflicting Adobe colour markers");
        adobeTransform = transform;
    }

    double at(int component, int x, int y) const {
        const Component& c = components[component];
        int sx = x * c.h / maxH, sy = y * c.v / maxV;
        return c.samples[std::size_t(sy) * c.stride + sx];
    }

public:
    explicit Decoder(const Bytes& data) : input{data, 0, data.size()} {
        double pi = std::acos(-1.0);
        for (int x = 0; x < 8; ++x)
            for (int u = 0; u < 8; ++u)
                cosine[x][u] = (u == 0 ? 1 / std::sqrt(2.0) : 1) *
                               std::cos((2 * x + 1) * u * pi / 16);
    }

    void decode() {
        require(input.byte() == 0xff && input.byte() == 0xd8, "not a JPEG file");
        for (;;) {
            int marker = input.marker();
            if (marker == 0xd9) {
                require(decoded, "image ends before a scan");
                input.finished();
                return;
            }
            require(marker != 0xd8 && !(marker >= 0xd0 && marker <= 0xd7),
                    "unexpected standalone marker");
            if (marker >= 0xc0 && marker <= 0xcf && marker != 0xc0 && marker != 0xc4)
                throw std::runtime_error("only baseline Huffman JPEG (SOF0) is supported");
            Reader segment = input.segment();
            if (marker == 0xc0) readFrame(segment);
            else if (marker == 0xdb) readQuant(segment);
            else if (marker == 0xc4) readHuffman(segment);
            else if (marker == 0xda) readScan(segment);
            else if (marker == 0xdd) {
                restartInterval = segment.word();
                segment.finished();
            } else if (marker >= 0xe0 && marker <= 0xef) readApp(marker, segment);
            else if (marker != 0xfe) throw std::runtime_error("unsupported JPEG marker");
        }
    }

    void write(const std::string& path) const {
        std::ofstream output(path, std::ios::binary);
        require(bool(output), "cannot open output file");
        output << "P6\n" << width << ' ' << height << "\n255\n";
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                double luminance = at(0, x, y);
                std::array<unsigned char, 3> rgb{};
                if (components.size() == 1) {
                    rgb.fill(sample(luminance));
                } else {
                    double cb = at(1, x, y) - 128, cr = at(2, x, y) - 128;
                    rgb = {sample(luminance + 1.402 * cr),
                           sample(luminance - 0.344136 * cb - 0.714136 * cr),
                           sample(luminance + 1.772 * cb)};
                }
                output.write(reinterpret_cast<const char*>(rgb.data()), rgb.size());
            }
        }
        output.close();
        require(bool(output), "failed while writing output file");
        std::cout << width << " x " << height << " -> " << path << '\n';
    }
};

Bytes readFile(const std::string& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    require(bool(input), "cannot open input file");
    std::streamoff size = input.tellg();
    require(size > 0 && size <= 32 * 1024 * 1024, "input limit: 32 MiB, nonempty file");
    Bytes data(static_cast<std::size_t>(size));
    input.seekg(0);
    input.read(reinterpret_cast<char*>(data.data()), size);
    require(bool(input), "failed while reading input file");
    return data;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: jpeg_decoder input.jpg output.ppm\n"
                     "8-bit baseline grayscale / YCbCr JPEG, single scan.\n";
        return 0;
    }
    if (argc != 3) {
        std::cerr << "Usage: jpeg_decoder input.jpg output.ppm\n";
        return 1;
    }
    try {
        std::error_code error;
        require(!std::filesystem::equivalent(argv[1], argv[2], error),
                "input and output must be different files");
        Bytes data = readFile(argv[1]);
        Decoder decoder(data);
        decoder.decode();
        decoder.write(argv[2]);
    } catch (const std::exception& error) {
        std::cerr << "jpeg_decoder: " << error.what() << '\n';
        return 1;
    }
}
