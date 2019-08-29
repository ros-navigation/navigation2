.. _template_publish:

Set up Publish directory and GitHub pages repository
####################################################

Once the template repository is :ref:`set up <guide>` on GitHub, it is time
to build and publish. 

.. note:: 

   We will be using the same working directory for publishing your
   documentation as we did in the setup guide:  ``otc-tcs-test``.

Create the GitHub Pages Repository 
==================================

#. Create a new repository on GitHub.com by clicking the new button from the
   project repositories tab.

   .. figure:: images/new-git-repository.png
      :width: 815px

#. Name your GitHub pages repository according to the following 
   convention ``<yourproject>.github.io`` and click ``create repository``.

   .. figure:: images/name-github-pages-repo.png
      :width: 665px

#. On the landing page for the empty repository click the hyperlink labeled
   ``README``.

   .. figure:: images/create-readme.png
      :width: 600px

#. Change the name of the file to ``.nojekyll``. There is no need to change
   the contents of the file.

   .. figure:: images/name-nojekyll.png
      :width: 785px


#. Scroll down and edit the commit message and click ``Commit new file``.

   .. figure:: images/commit-nojekyll.png
      :width: 700px

Clone your GitHub pages repository to your local working directory
==================================================================

#. Create a directory called ``website`` inside of ``otc-tcs-test``.

   .. code-block:: console

      user@yourcomputer:~/otc-tcs-test$ mkdir website

   At this point, the contents of your ``otc-tcs-test`` directory should
   look like this:

   .. code-block:: console

      user@yourcomputer:~/otc-tcs-test$ ls -al
      total 0
      drwxrwxrwx 0 root root 512 Jul 29 16:54 .
      drwxrwxrwx 0 root root 512 Jul 27 16:46 ..
      drwxrwxrwx 0 root root 512 Jul 27 16:49 doc-repo
      drwxrwxrwx 0 root root 512 Jul 29 16:54 website

#. Click ``Clone or download`` on the repository ``<> Code`` tab.

   .. figure:: images/clone-repo.png
      :width: 815px

#. Copy the URL of the respository by clicking the clipboard icon.

   .. figure:: images/copy-githubio-clone-url.png 
      :width: 400px

#. From the ``otc-tcs-test`` directory clone the GitHub pages repository to
   your ``website`` directory.

   .. code-block:: console

      user@yourcomputer:~/otc-tcs-test$ git clone git@github.com:yourproject/yourproject.github.io.git website

   The contents of ``otc-tcs-test`` should now look like this:

   .. code-block:: console

      user@yourcomputer:~/otc-tcs-test/$ cd website
      user@yourcomputer:~/otc-tcs-test/website$ ls -al
      total 0
      drwxrwxrwx 0 root root 512 Jul 29 21:43 .
      drwxrwxrwx 0 root root 512 Jul 29 16:54 ..
      drwxrwxrwx 0 root root 512 Jul 29 21:43 .git
      -rwxrwxrwx 1 root root  24 Jul 29 21:43 .nojekyll

For building and deployment instructions go to :ref:`install_doc_gen`. 