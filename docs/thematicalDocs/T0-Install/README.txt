A remark before actually including the SoftRobos plugin in your build: Please chose a directory external to your Sofa sources to download the SoftRobots plugin. Your folder structure might look something like this:

/home/user/sofa/build --> Folder where Sofa is built
/home/user/sofa/src --> Folder containing the sources for Sofa, i.e. the root directory of the git-repo
/home/user/sofa/plugins/SoftRobots --> Folder containing the sources for the SoftRobots plugin

To compile the plugin it is necessary to direct the Sofa build process to include the directory of the SoftRobots plugin. Actually, it is a single step using cmake. Just set the cmake-variable 'SOFA_EXTERNAL_DIRECTORIES' to the path where the SoftRobots plugin is placed (=/home/user/sofa/plugins/SoftRobots). This can be done using cmake-gui for example. 'Configure' and 'generate' the build and the compile.  

