function [stat] = PrintResult(fnameStats,sceneName,outputfile)
fid = fopen (outputfile, "a+");

stat = ReadResults(fnameStats);

for r = 1:size(stat,1)
  fprintf(fid, '%s ', sceneName);
  for i = 1:size(stat,2)
      fprintf(fid, '%f ',stat(r,i));
  end
  fprintf(fid, '\n');
end

fclose (fid);
end
