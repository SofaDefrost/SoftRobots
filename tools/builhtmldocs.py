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
	                	line=line.replace(aString, "<a href=\"" + dictionary[aString] + "\">" + aString + "</a>")
        	fo.write(line)
        			
        fo.close()
        f.close()

if len(sys.argv) <= 2:
	print ("USAGE: ./buildhtmldocs dirname componentDirName")
	sys.exit(-1)


dictionary={}

for aPathComponent in sys.argv[2:]:
	for (dirpath, dirnames, aFilenames) in os.walk(aPathComponent):
	        for aFilename in aFilenames:
			aFilename,theExt = os.path.splitext(ntpath.basename(aFilename))
			if theExt in sofaext: 	
				dictionary[aFilename] = dirpath+"/"+aFilename+theExt


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
                        print("Generating: " + dirpath + aFile + ".html from " +
                         dirpath+aFile+ext )
                        retcode = subprocess.call(["pandoc", dirpath + "/" +aFilename, "-s", "-c", "https://gist.githubusercontent.com/ryangray/1882525/raw/2a6e53f645b960f0bed16d686ba3df36505f839f/buttondown.css", "-o", dirpath + "/" + aFile + ".html.tmp"])
                        if retcode == 0 :
                        	print("Replacing token...")
                                replaceStringInFile(dirpath + "/" + aFile + ".html.tmp", dirpath + "/" + aFile + ".html", dictionary)
                                os.remove( dirpath + "/" + aFile + ".html.tmp" )
