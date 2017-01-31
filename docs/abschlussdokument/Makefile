.PHONY: all clean open

all: 
	pdflatex paper.tex
	bibtex paper
	pdflatex paper.tex
	pdflatex paper.tex
clean:
	rm -f *.toc
	rm -f *.pdf
	rm -f *.log
	rm -f *.out
	rm -f *.aux
	rm -f *.bbl
	rm -f *.blg
	rm -f *.brf

open:
	zathura ba.pdf
