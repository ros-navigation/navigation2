.. _important-sphinx-files:

Important Sphinx files
######################

These files will appear in most Sphinx projects and are used to extend and define the build behavior.

.. rst-class:: content-collapse

conf.py
=======

Configure Sphinx variables

suffix
    Set the kind of files to process (.rst or .md).

    .. code-block:: python

       source_suffix = '.rst'

master document
    Set the file to use as the top level document for the project (relative to conf.py).

    .. code-block:: python

        master_doc = 'index'

general info
    General information about the project.

    .. code-block:: python

        project = u'Open source documentation template'
        copyright = u'2018, TCS Documentation Template'
        author = u'TCS developers'

extensions
    Add functionality to your Sphinx build.

    .. code-block:: python

        extensions = ['breathe', 'sphinx.ext.graphviz', 'sphinxcontrib.plantuml']


exclude patterns
    Exclude directories from Sphinx processing by adding them to a list.

    .. code-block:: python

        exclude_patterns = ['_build','_themes','scripts' ]

theme
    Use one of the provided Sphinx themes or a custom one.

    .. code-block:: python

        html_theme_path = ['_themes']
        html_theme = 'otc_tcs_sphinx_theme'

.. rst-class:: content-collapse

substitutions.txt
=================

Add content to this file when you want it to appear at the end of or be
accessed by every page of the project. We use it to set global string
replacements and links.

replace string
    Useful when you want to create a shorthand version of a long string.

    .. code-block:: rest

        .. |PN| replace:: project name

global link
    Use these when you will be re-using a link throughout the project.

    .. code-block:: rest

        .. _project website: https://rosplanning.github.io/navigation2/

unicode
    You can also create shortcuts for commonly used unicode characters.

    .. code-block:: rest

        .. |copy|   unicode:: U+000A9 .. COPYRIGHT SIGN
            :ltrim:

.. note::

    We implmented this by using an ``include`` directive in
    the ``rst_epilog`` variable in :file:`conf.py` like this:

    .. code-block:: python

        rst_epilog = """
        .. include:: /substitutions.txt
        """
