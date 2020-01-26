
all: presentation_2020-01-28.pdf

%.pdf: %.tex
	mkdir -p ./$*-build/
	latexmk -pdflatex=lualatex -pdf -jobname=./$*-build/$* $<
