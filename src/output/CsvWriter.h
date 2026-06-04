#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>

// Simple CSV writer.
// Opens the file on construction; each call to writeRow() appends a row.
// Columns are comma-separated; strings with commas or quotes are quoted.
class CsvWriter {
public:
    explicit CsvWriter(const std::string& path) : path_(path) {
        file_.open(path);
        if (!file_.is_open())
            throw std::runtime_error("CsvWriter: cannot open " + path);
    }

    ~CsvWriter() { if (file_.is_open()) file_.close(); }

    CsvWriter(const CsvWriter&)            = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;

    void writeHeader(const std::vector<std::string>& cols) {
        writeLine(cols);
    }

    void writeRow(const std::vector<std::string>& values) {
        writeLine(values);
    }

    // Convenience: build a row from mixed types
    template<typename... Args>
    void writeRowV(Args&&... args) {
        std::vector<std::string> row;
        row.reserve(sizeof...(args));
        (row.push_back(toStr(std::forward<Args>(args))), ...);
        writeRow(row);
    }

    void flush() { file_.flush(); }

private:
    std::string   path_;
    std::ofstream file_;

    static std::string escape(const std::string& s) {
        if (s.find_first_of(",\"\n") == std::string::npos) return s;
        return "\"" + s + "\"";
    }

    void writeLine(const std::vector<std::string>& cols) {
        for (std::size_t i = 0; i < cols.size(); ++i) {
            if (i > 0) file_ << ',';
            file_ << escape(cols[i]);
        }
        file_ << '\n';
    }

    template<typename T>
    static std::string toStr(T&& v) {
        if constexpr (std::is_same_v<std::decay_t<T>, std::string>) {
            return v;
        } else if constexpr (std::is_floating_point_v<std::decay_t<T>>) {
            std::ostringstream oss;
            // Use 'g' format: switches to scientific notation for very small/large
            // values, avoiding misleading "0.000000" for sub-micron quantities.
            oss.precision(10);
            oss << v;
            return oss.str();
        } else {
            return std::to_string(v);
        }
    }
};
