#ifndef CREO2URDF_JOINT_CSV_H
#define CREO2URDF_JOINT_CSV_H

#include <rapidcsv.h>
#include <stdexcept>
#include <string>

/**
 * @brief Read a numeric joint parameter, preserving RapidCSV conversion semantics.
 * @param csv Parsed joint table with named rows and columns.
 * @param filePath Source CSV path, included in diagnostics.
 * @param jointName Final URDF joint name identifying the row.
 * @param columnName Parameter identifying the column.
 * @return The numeric cell value.
 * @throws std::runtime_error If the cell is missing or cannot be converted;
 * the diagnostic identifies the file, joint, column and available raw value.
 */
inline double readJointCsvNumber(const rapidcsv::Document& csv, const std::string& filePath, const std::string& jointName, const std::string& columnName) {
    const auto context = "CSV '" + filePath + "', joint '" + jointName + "', column '" + columnName + "'";
    std::string rawValue;
    try {
        rawValue = csv.GetCell<std::string>(columnName, jointName);
    } catch (const std::exception& error) {
        throw std::runtime_error("Cannot read " + context + ": " + error.what());
    }
    try {
        return csv.GetCell<double>(columnName, jointName);
    } catch (const std::invalid_argument&) {
        throw std::runtime_error("Invalid numeric value in " + context + ": '" + rawValue + "' (expected a number)");
    } catch (const std::out_of_range&) {
        throw std::runtime_error("Numeric value out of range in " + context + ": '" + rawValue + "'");
    }
}

/**
 * @brief Read an optional limit without converting empty or whitespace-only cells.
 * @param csv Parsed joint table.
 * @param filePath Source path used in conversion diagnostics.
 * @param jointName Row containing the joint parameters.
 * @param columnName Optional limit column.
 * @param[out] value Numeric limit; unchanged when the column or value is absent.
 * @return True for a supplied numeric value, false for an absent column or blank cell.
 * @throws std::runtime_error If a supplied value cannot be converted to a number.
 */
inline bool readOptionalJointCsvNumber(const rapidcsv::Document& csv, const std::string& filePath, const std::string& jointName, const std::string& columnName, double& value) {
    if (csv.GetColumnIdx(columnName) < 0) {
        return false;
    }
    const auto rawValue = csv.GetCell<std::string>(columnName, jointName);
    if (rawValue.find_first_not_of(" \t\r\n\f\v") == std::string::npos) {
        return false;
    }
    value = readJointCsvNumber(csv, filePath, jointName, columnName);
    return true;
}

#endif
