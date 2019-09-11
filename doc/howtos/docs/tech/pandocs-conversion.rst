.. _pandocs-conversion:

Pandocs Conversion Tutorial
###########################

.. contents:: :local:
   :depth: 2


This tutorial shows you how to use ``pandoc`` to convert files in various
formats.

Table 1 shows current conversions.

.. list-table:: Table 1. Conversions
   :widths: 80 100 100
   :header-rows: 1
   :stub-columns: 1

   * - Convert
     - From
     - To
   * - reSTructured Text to HTML
     - ``.rst``
     - ``.html``
   * - Markdown to reSTructured Text
     - ``.md``
     - ``.rst``
   * - reSTructured Text to HTML
     - ``.rst``
     - ``.html``

Scope
=====

Conversion should be a preliminary step in migrating to Sphinx
for your documentation project. The results of conversion will complete
about 80% of the work. It's expected that you'll edit converted files as
post-processing. We recommend studying the `Pandoc website`_ to learn more.

In the Clear Linux\* documention, we established
`Documentation Contribution Guidelines`_, which explain conventions and
usage of reST syntax. We encourage you to establish similar guidelines for
your team. Even minimal initial investment in guidelines greatly
reduces future work. More importantly, providing clear guidelines supports
your team's long-term maintence efforts.

Note: See also the Clear Linux `reSTructured Text Guide`_.

Prerequisites
=============

* `Install Pandocs`_
* Command line interface basic skills
* Text editor (e.g., Sublime, Notepad++, etc.)
* Web browser

Introduction
============
We recommend following reST conventions and best practices for Sphinx. To
learn more, visit the `Sphinx website`_

Single file Conversion
=======================

For single file conversions,


cd into the file directory:

.. code-block:: bash

   $ cd pandocs-tutorial


From Markdown to reST
---------------------

#. Run the command:

.. code-block:: bash

   $ pandoc --from=markdown --to=rst --output=cicero.rst cicero.md

From Markdown to HTML
---------------------

#. Run the command:

.. code-block:: bash

   $ pandoc cicero.md -f markdown -t html -s -o cicero.html

From reST to HTML
-----------------
#. Run the command:

.. code-block:: bash

   $ pandoc -f rst -t html cicero.rst > cicero.html

**Congratulations!** You have learned how to convert single files to your preferred format.


.. _Install Pandocs: https://pandoc.org/installing.html

.. _Sphinx website: http://www.sphinx-doc.org/en/master/index.html

.. _Pandoc website: https://pandoc.org/

.. _Documentation Contribution Guidelines: https://clearlinux.org/documentation/clear-linux/reference/collaboration/documentation

.. _reSTructured Text Guide: https://clearlinux.org/documentation/clear-linux/reference/collaboration/documentation/rest#additional-information
