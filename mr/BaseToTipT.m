function T = BaseToTipT(in)
    sz=size(in);
    if (size(sz,2)==3)
        T=in(:,:,1);
        for i = 2:sz(3)
            T=T*in(:,:,i);
        end
    else
        T=in;
    end
end