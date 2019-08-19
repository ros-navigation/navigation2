FROM alpine:latest

#ENV http_proxy <myproxy>
#ENV https_proxy <myproxy>

RUN apk --no-cache add python3
RUN python3 -m ensurepip
RUN pip3 install sphinx==1.7.5 docutils==0.14 sphinx_rtd_theme breathe==4.9.1 sphinxcontrib-plantuml
RUN apk --no-cache add make
RUN apk --no-cache add doxygen
RUN apk --no-cache add graphviz
RUN apk --no-cache add ttf-dejavu
RUN apk --no-cache add openjdk8-jre
