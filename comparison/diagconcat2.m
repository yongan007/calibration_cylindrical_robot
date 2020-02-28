function t = diagconcat2(a,b)
   [am an] = size(a);
   [bm bn] = size(b);
   t = zeros(am+bm, an+bn);
   t(1:am,1:an) = a;
   t((am+1):(am+bm),(an+1):(an+bn)) = b;
end