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
import re
sofaext=['.scn', ".pyscn", ".psl"]

def replaceStringInFile(aFile, outFile, aDictionary):
        f = open(aFile, "rt")
        fo = open(outFile, "w")
        lineno=1
        for line in f:
                for aString in aDictionary:
                    if re.search(aString+"(?!::)", line) != None:
                        line = re.sub(aString+"(?!=::)", "<a href=\"" + dictionary[aString]["url"] + "\">" + dictionary[aString]["name"] + "</a>", line)

                if re.search("..autolink::", line):
                    print("Missing autolink in line "+str(lineno)+" : "+line)

                lineno += 1

                #if aString in line:
	        #	line=line.replace(aString, "<a href=\"" + dictionary[aString] + "\">" + aString + "</a>")
        	fo.write(line)
        			
        fo.close()
        f.close()

if len(sys.argv) <= 2:
	print ("USAGE: ./buildhtmldocs dirname componentDirName")
	sys.exit(-1)


dictionary={}

#for aPathComponent in sys.argv[2:]:
#	for (dirpath, dirnames, aFilenames) in os.walk(aPathComponent):
#	        for aFilename in aFilenames:
#			aFilename,theExt = os.path.splitext(ntpath.basename(aFilename))
#			if theExt in sofaext:
#				dictionary[aFilename] = dirpath+"/"+aFilename+theExt


print(str(len(dictionary))+" components loaded.")

hooks = "hooks.json"
if os.path.exists(hooks):
	d = json.load(open(hooks))
	for k in d:
		dictionary[k] = d[k]

	print(str(len(d))+" hooks loaded.")


pathprefix = os.path.abspath(sys.argv[1]) + "/"
for (dirpath, dirnames, aFilenames) in os.walk(pathprefix):
        dirpath = os.path.abspath(dirpath) + "/"
        for aFilename in aFilenames:
                aFile, ext = os.path.splitext(aFilename)
		if ext in [".md"]:
                        print("Generating: " + dirpath + aFile + ".html from " +
                        dirpath+aFile+ext )
                        os.chdir(dirpath)
                        relpathstyle = os.path.relpath(pathprefix, dirpath)
                        retcode = subprocess.call(["pandoc", dirpath + "/" +aFilename, "-s", "-c", relpathstyle+"/docs/style.css", "-o", dirpath + "/" + aFile + ".html.tmp"])
                        if retcode == 0 :
                        	replaceStringInFile(dirpath + "/" + aFile + ".html.tmp", dirpath + "/" + aFile + ".html", dictionary)
                                os.remove( dirpath + "/" + aFile + ".html.tmp" )
