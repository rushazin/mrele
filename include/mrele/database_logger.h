// include/mrele/database_logger.h
#ifndef MRELE_DATABASE_LOGGER_H
#define MRELE_DATABASE_LOGGER_H

extern "C" {
    #include "..//..//sqlite3.h"
}
class DatabaseLogger {
private:
    sqlite3* db{};
public:
    DatabaseLogger();
    ~DatabaseLogger();
    void logStep(double a1, double a2, double ex, double ey, bool hit) const;
    void printLogs();

    void printFinalKPI();
};

#endif