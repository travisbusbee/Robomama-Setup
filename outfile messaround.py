#def setup(self):
#        """ Set the environment into a consistent state to start off. This
#        method must be called before any other commands.
#
#        """
#        outfile = self.outfile
#        if outfile is not None and self.aerotech_include is True:
#            if isinstance(outfile, basestring):
#                outfile = open(outfile, 'w+')  # open it if it is a path
#            self.outfile = outfile
#            lines = open(os.path.join(HERE, 'header.txt')).readlines()
#            outfile.writelines(lines)
#            outfile.write('\n')
#            if self.header is not None:
#                lines = open(self.header).readlines()
#                outfile.writelines(lines)
#                outfile.write('\n')
                
outfile = 'C:\Users\Lewis Group\Desktop\outfile_test.txt'
lines = 'test this'
outfile = open(outfile, 'w+')
outfile.writelines(lines)
outfile.write('\n')
lines = 'or not'
outfile.writelines(lines)
outfile.close()