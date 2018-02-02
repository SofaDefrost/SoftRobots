#!/usr/bin/python
# -*- coding: utf-8 -*-
# A small tool to generate .html from the .md contains in the plugin
# 
# Contributors:
#  	damien.marchal@univ-lille1.fr
#
import os
import subprocess
import sys
import re
import ntpath
import json 

sofaext=['.scn', ".pyscn", ".psl"]

def replaceStringInFile(aFile, outFile, aDictionary):
        f = open(aFile, "rt")
        fo = open(outFile, "w")
        for line in f:
                for aString in aDictionary:
                	if aString in line and '<a href="' not in line:
                                aRegex=r"\s" + re.escape(aString) + r"\s"
                                line = re.sub(aRegex, " [" + aString + "](" + dictionary[aString] + ") ",line)
        	fo.write(line)
        fo.close()
        f.close()

def createDictionaryOfComponentsFrom(aDirectory):
        aDictionary={}
        for aPathComponent in sys.argv[2:]:
	        for (dirpath, dirnames, aFilenames) in os.walk(aPathComponent):
	                for aFilename in aFilenames:
			        aFilename,theExt = os.path.splitext(ntpath.basename(aFilename))
			        if theExt in sofaext:
				        aDictionary[aFilename] = dirpath+"/"+aFilename+theExt
        return aDictionary

############################################ MAIN HERE

if len(sys.argv) <= 2:
	print ("USAGE: ./buildhtmldocs dirname componentDirName")
	sys.exit(-1)

dictionary=createDictionaryOfComponentsFrom(sys.argv[2])

print(str(len(dictionary))+" components loaded.")

hooks = "hooks.json"
if os.path.exists(hooks):
	d = json.load(open(hooks))
	for k in d:
		dictionary[k] = d[k]["url"]

	print(str(len(d))+" hooks loaded.")

pathprefix = sys.argv[1]
for (dirpath, dirnames, aFilenames) in os.walk(pathprefix):
        for aFilename in aFilenames:
                aFile, ext = os.path.splitext(aFilename)
		if ext in [".md"]:
                        tmpFile=dirpath + "/" +  aFilename + ".tmp"
                        finalFile= dirpath + "/" + aFile + ".html"
                        replaceStringInFile(dirpath + "/" + aFilename, tmpFile, dictionary)
                        print("Generating: " + dirpath + aFile + ".html from " + dirpath+aFile+ext )
                        retcode = subprocess.call(["pandoc", tmpFile, "-s", "-c", "https://gist.githubusercontent.com/ryangray/1882525/raw/2a6e53f645b960f0bed16d686ba3df36505f839f/buttondown.css", "-o", finalFile])
                        if retcode != 0 :
                        	print("Error during the generation of " + finalFile)
                        os.remove(tmpFile)
