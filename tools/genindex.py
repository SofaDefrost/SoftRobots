# -*- coding: utf-8 -*-
# A small tool to generate .html containing the list of Sofa scenes in directory tree. 
# 
# Contributors:
#  	damien.marchal@univ-lille1.fr
import os
import sys


def addToPrefixDictionary(d, path, filename):
	s=d
	entries=relpath.split(os.path.sep) 			
	for entry in entries:
		if entry in s:
			s = s[entry]
		elif filename == entry:
			s[entry] = (path, filename)
		else:
			s[entry] = {}
			s = s[entry]			
	return d


def printList(f, k, d, indent=0):
	if isinstance(d, dict):
		if len(d) == 1:
			#f.write("\n")
			for i in range(indent):
				f.write("\t")
			f.write("- "+str(k)+"\n")
			for i in d:				
				printList(f,k+os.path.sep+i, d[i],indent+1)
		else:
			if k != "":
				#f.write("\n")
				for i in range(indent):
					f.write("\t")
				f.write("- "+str(k)+"\n")
				for i in d:	
					printList(f, i, d[i],indent+1)
			else:
				for i in d:	
					printList(f, i, d[i],indent)
			
	else:
		for i in range(indent):
			f.write("\t")
		f.write("- ["+d[1]+"]("+d[0]+")\n")

if len(sys.argv) != 3:
	print("USAGE: genindex.py ProjectName destfile.md")
	sys.exit(-1)	
		
pathprefix = os.getcwd() 
d = {}
for (dirpath, dirnames, filenames) in os.walk(pathprefix):
	for filename in filenames:
		if os.path.splitext(filename)[1] in [".pyscn", "scn", "psl", "pslx"]:
			relpath = os.path.relpath(os.path.join(dirpath, filename))
			addToPrefixDictionary(d, relpath, filename)

f = open(sys.argv[2], "wt")			
f.write("# "+sys.argv[1]+"\n")
printList(f, "", d)
f.close()
