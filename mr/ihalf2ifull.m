function out=ihalf2ifull(idia, icorner)
    out=[idia(1) icorner(1) icorner(2); icorner(1) idia(2) icorner(3); icorner(2) icorner(3) idia(3)];
end

