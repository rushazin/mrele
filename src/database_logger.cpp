// src/database_logger.cpp
#include "mrele/database_logger.h"
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

extern "C" {
#include "..//sqlite3.h"
}

int sqliteCallback(void* NotUsed, int argc, char** argv, char** azColName) {
    for (int i = 0; i < argc; i++) {
        std::cout << azColName[i] << ": " << (argv[i] ? argv[i] : "NULL") << " | ";
    }
    std::cout << "\n";
    return 0;
}

DatabaseLogger::DatabaseLogger() {
    sqlite3_open("telemetry.db", &db);
    const char* sql = "CREATE TABLE IF NOT EXISTS Logs("
                      "ID INTEGER PRIMARY KEY AUTOINCREMENT, "
                      "Angle1 REAL, Angle2 REAL, "
                      "ErrorX REAL, ErrorY REAL, Hit INT);";
    sqlite3_exec(db, sql, nullptr, nullptr, nullptr);
}

DatabaseLogger::~DatabaseLogger() {
    sqlite3_close(db);
}

void DatabaseLogger::logStep(double a1, double a2, double ex, double ey, bool hit) const {
    std::string sql = "INSERT INTO Logs (Angle1, Angle2, ErrorX, ErrorY, Hit) VALUES (" +
                      std::to_string(a1) + ", " + std::to_string(a2) + ", " +
                      std::to_string(ex) + ", " + std::to_string(ey) + ", " +
                      std::to_string(hit ? 1 : 0) + ");";
    sqlite3_exec(db, sql.c_str(), nullptr, nullptr, nullptr);
}

void DatabaseLogger::printLogs() {
    printFinalKPI();
}

void DatabaseLogger::printFinalKPI() {
    std::string db_name = "telemetry.db";
    sqlite3_close(db);

    if (sqlite3_open(db_name.c_str(), &db) != SQLITE_OK) {
        std::cout << "[Analytics Error]: Cannot reopen database file.\n";
        return;
    }

    sqlite3_stmt* stmt;
    const char* sql = "SELECT ID, ErrorX, ErrorY, Hit FROM Logs ORDER BY ID;";

    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        std::cout << "[Analytics Error]: Cannot prepare SQL statement. Error: " << sqlite3_errmsg(db) << "\n";
        return;
    }

    int total_steps = 0;
    int hit_steps = 0;
    double sum_squared_errors = 0.0;
    double settling_time = -1.0;
    bool locked = false;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        int id = sqlite3_column_int(stmt, 0);
        double ex = sqlite3_column_double(stmt, 1);
        double ey = sqlite3_column_double(stmt, 2);
        int hit = sqlite3_column_int(stmt, 3);

        total_steps++;
        if (hit == 1) {
            hit_steps++;
        }

        sum_squared_errors += (ex * ex + ey * ey);

        double current_time = id * 0.01;
        if (!locked && std::abs(ex) < 0.99 && std::abs(ey) < 0.99) {
            settling_time = current_time;
            locked = true;
        }
    }
    sqlite3_finalize(stmt);

    if (total_steps == 0) {
        std::cout << "[Analytics]: Database is empty. No KPI calculated.\n";
        return;
    }

    double lock_percentage = (static_cast<double>(hit_steps) / static_cast<double>(total_steps)) * 100.0;
    double rms_error = std::sqrt(sum_squared_errors / static_cast<double>(total_steps));

    std::cout << "\n--------------------------------------------------\n";
    std::cout << "         CAP SYSTEM PERFORMANCE REPORT (KPI)      \n";
    std::cout << "--------------------------------------------------\n";
    if (settling_time >= 0.0) {
        std::cout << " Time to First Lock (Settling Time) : " << settling_time << " sec\n";
    } else {
        std::cout << " Time to First Lock (Settling Time) : Target NOT captured\n";
    }
    std::cout << " Beam Retention Rate (Lock Ratio)   : " << lock_percentage << " %\n";
    std::cout << " Root Mean Square Error (RMS QPD)   : " << rms_error << "\n";
    std::cout << "--------------------------------------------------\n\n";
}