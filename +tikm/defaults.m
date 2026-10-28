function s = defaults(s,base)
%DEFAULTS Fill missing options without overriding caller settings.
keys = fieldnames(base);
for k=1:numel(keys)
    if ~isfield(s,keys{k}), s.(keys{k})=base.(keys{k}); end
end
end
