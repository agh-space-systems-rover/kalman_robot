#include <string>
#include <iostream>
#include <libgen.h>
#include <cstring>
#include <unistd.h>
#include <linux/limits.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>

// Kids, do not do this in production!
int main() {
    const char * this_file_path{__FILE__};
    char* this_file_path_copy = strdupa(this_file_path);

    char* kalman_groundstation_path = this_file_path_copy;
    for (size_t i = 0; i < 3; i++)
    {
        kalman_groundstation_path = dirname(kalman_groundstation_path);
    }
    // Now, kalman_groundstation_path is <path_to_kalman_robot>/kalman_groundstation
    char * kalman_groundstation_str = basename(kalman_groundstation_path);
    if (strcmp(kalman_groundstation_str, "kalman_groundstation")){
        return 1;
    }

    char* web_frontend_path = (char*) alloca(PATH_MAX + 1);
    snprintf(web_frontend_path, PATH_MAX + 1, "%s/web_frontend", kalman_groundstation_path);
    DIR * web_frontend_dir = opendir(web_frontend_path);
    if (web_frontend_dir){
        closedir(web_frontend_dir);
    } else {
        return 1;
    }

    // Actually run the frontend
    chdir(web_frontend_path);
    system("npm start");

    return 0;
}