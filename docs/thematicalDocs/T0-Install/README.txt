
--- Directory structure, before compiling ---

A remark before actually including the SoftRobos plugin in your build: Please chose a directory external to your Sofa sources to download the SoftRobots plugin. Specifically, don't try to use the existing folder applications/plugins in Sofa, because it will potentially mess up your git-repos. Therefore, your folder structure should look something like this:

/home/user/sofa/build --> Folder where Sofa is built
/home/user/sofa/src --> Folder containing the sources for Sofa, i.e. the root directory of the git-repo
/home/user/sofa/plugins/SoftRobots --> Folder containing the sources for the SoftRobots plugin

--- Compiling ---

To compile the plugin it is necessary to direct the Sofa build process to include the directory of the SoftRobots plugin. Actually, it is a single step using cmake. Just set the cmake-variable 'SOFA_EXTERNAL_DIRECTORIES' to the path where the SoftRobots plugin is placed (=/home/user/sofa/plugins/SoftRobots). This can be done using cmake-gui for example. The variable can be found using the search function of the cmake-gui. 'Configure' and 'generate' with cmake and then compile (run make or ninja, etc. in /home/user/sofa/build).  

