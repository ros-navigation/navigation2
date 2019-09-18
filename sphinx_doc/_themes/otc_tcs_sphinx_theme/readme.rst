OTC-TCS Sphinx Theme
####################

Built on top of the popular Read the Docs Sphinx theme, this theme
has a few small formatting/color improvements and implements collapsible
sections. It is in active development in order to support OTC and other
Intel organizations to publish high quality, consistent documentation
for open source projects.

How to use it
*************

#. Download or clone the repository
#. Create a ``_themes`` directory in main directory of your sphinx
   documentation
#. Install sphinx_rtd_theme using pip: ``pip3 install sphinx_rtd_theme``
#. copy ``otc_tcs_sphinx_theme`` directory into the new ``_themes`` directory
#. Add the following to your ``conf.py``:

   .. code-block:: python

       html_theme = 'otc_tcs_sphinx_theme'
       html_theme_path = ['_themes']
