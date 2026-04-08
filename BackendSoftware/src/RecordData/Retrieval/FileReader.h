class FileReader
{
    public:
        /**
        * @brief Construct FileReader
        * @param backend Storage backend to read from (not owned by FileReader)
        */
        FileReader(IStorage *backend) : _backend(backend) {} // using an initializer list to create an IStorage object

        /**
        * @brief Print entire file to Serial
        * @param filename Path to file
        * @return true if file was read successfully
        */
        bool printFile(const char *filename);

        /**
        * @brief Read file line by line with callback
        * @param filename Path to file
        * @param callback Function called for each line
        * @return true if file was read successfully
        */
        bool readLines(const char *filename, void (*callback)(const char *line));

        /**
        * @brief Check if file exists
        * @param filename Path to file
        * @return true if file exists
        */
        bool exists(const char *filename);

        /**
        * @brief Delete a file
        * @param filename Path to file
        * @return true if file was deleted successfully
        */
        bool deleteFile(const char *filename);

        /**
        * @brief Interactive command-line interface
        *
        * Commands:
        * - cmd/sf <filename> : Show file contents
        * - cmd/rm <filename> : Remove file
        * - cmd/help : Show help
        * - cmd/quit : Exit
        */
        void handleCommands();

};