# --- picknames.py ---
import os
homedir = os.getcwd()
filenames=os.listdir(os.getcwd())
out=open('names.txt','w')
for name in filenames:
     out.write('"' + homedir + "\\" + name + '"' + ',' + '\n')
out.close()
