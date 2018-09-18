%
% uoitthesis v1.0
% By Wendy (WenXiang) Yang <wyang@unb.ca>
% Version 1.0 released 2005/11/10
%-------------------------- identification ---------------------
%\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{uoitthesis}[2005/11/10 v1.0 UOIT LaTex Thesis class]
%-------------------------- Declaration of options -----------------------
%This class uses a report standard class
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}
\ProcessOptions\relax \LoadClass{report}
%%-------------------page setup-----------------------------------------
\usepackage[left=3.8cm, right=2.5cm, top=2.5cm, bottom=2.5cm]{geometry}
%
%%---------------------define command in the title page--------------------
\newcommand{\submittext}{A THESIS SUBMITTED IN PARTIAL FULFILLMENT OF THE REQUIREMENTS FOR THE DEGREE OF}
\newcommand{\thesisnote}{by}
%\newcommand{\gaunote}{In the Graduate Academic Unit of }
\newcommand{\gaunote}{in}
\newcommand{\acceptnote}{This thesis is accepted}
\newcommand{\dean}{Dean of Graduate Studies}
\newcommand{\school}{THE UNIVERSITY OF ONTARIO INSTITUTE OF TECHNOLOGY}
\newcommand{\abstracter}{\begin{flushleft} \huge{\textbf{Abstract}} \end{flushleft}}

%
%
% DECLARATIONS
%
% These macros are used to declare arguments needed for the
% construction of the title page and other preamble.
%
% candidate's previous degree
\def\predegree#1{\gdef\@predegree{#1}}
%
% candidate's previous degree
\def\predegrees#1{\gdef\@predegrees{#1}}
%
% The full (unabbreviated) name of the degree
\def\degreename#1{\gdef\@degree{#1}}
%
% The name of your Graduate Academic Unit(eg. Computer Science, Maths, Physics)
\def\gau#1{\gdef\@gau{#1}}
%
% The name of your program (Mechanical Engineering, etc)
\def\program#1{\gdef\@program{#1}}

%your supervisor including name, degree department or field
\def\supervisor#1{\gdef\@supervisor{#1}}
%
%examining board including name, degree department or field starting from Chair
\def\examboard#1{\gdef\@examboard{#1}}
%
%external examiner including name, degree, department or field and Institution
\def\externalexam#1{\gdef\@externalexam{#1}}
%
%copyright including name of candiate and year of graduation
\def\copyrightyear#1{\gdef\@copyrightyear{#1}}
%define title page layout
%
\newcommand{\uoittitlepage}{%
\thispagestyle{empty}%
\begin{center}
\vskip 4in
\normalfont\Large\bfseries\@title\\
\vskip 1em
\normalfont\normalsize\thesisnote\\
\vskip 1em
\normalfont\normalsize\@author\\
\vskip 2em
\normalfont\normalsize\@predegree\\
\vskip 2em
\normalfont\normalsize\@predegrees\\
\vskip 2em
\normalfont\normalsize\submittext\\
\vskip 2em
\normalfont\normalsize\@degree\\
\vskip 2em
\normalfont\normalsize\gaunote\\ \vskip 1em \@gau\\
\vskip 2em
\normalfont\normalsize\@program
\vskip 2em
\end{center}
\begin{table}[!h]
\begin{tabular}{l l}
Supervisor(s): & \@supervisor \\
Examining Board: & \@examboard \\
External Examiner: & \@externalexam
%\@externalexam
\end{tabular}
\end{table}
\begin{center}
%\normalfont\normalsize\acceptnote\\
%\vskip 2em
%\normalfont\normalsize\dean\\
\vskip 2em
\normalfont\normalsize\school\\
\vskip 2em
\normalfont\normalsize\@date\\
\vskip 2em
\normalfont\normalsize\copyright \@author ~ \@copyrightyear
\end{center}}
%-----------end of Title page--------------------

