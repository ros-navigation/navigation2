.. _contribute:

Getting Involved
################

As an open-source project, we welcome and encourage the community to
submit patches directly to the |LPN|.  In our collaborative open
source environment, standards and methods for submitting changes help
reduce the chaos that can result from an active development community.

This document explains how to participate in project conversations, log
and track bugs and enhancement requests, and submit patches to the
project so your patch will be accepted quickly in the codebase.

Getting Involved
****************

If you're interested in getting involved in Navigation 2, first of all, welcome!
We encourage everyone to get involved from students, to junior developers, to senior developers, and executives.
There's something to do for everyone from bug fixes, to feature development, new algorithms, and refactoring.

All ROS2 TSC Working Groups have their meetings on the `working group calendar <https://calendar.google.com/calendar/embed?src=agf3kajirket8khktupm9go748%40group.calendar.google.com>`_.
Here, you can find the date and time of the Navigation2 working group meeting. Make sure you're checking in your local timezone.
From this calendar, you can add yourself to the event so it will appear on your google calendar and get the event link to the call through Google Hangouts.
We encourage everyone interested to come to the meeting to introduce yourself, your project, and see what everyone is working on.

Further, `ROS Discourse <https://discourse.ros.org/>`_ is a good place to follow larger discussions happening in the community and announcements. This is **not** the correct place to post questions or ask for assistance. Please visit `ROS Answers <https://answers.ros.org/>`_ for Q&A.

If you're looking to contribute code or bugs, please see the Process section below.

Over time, for developers that have an interest and have shown technical competence in an area of the stack, we elevate developers to a maintainers status.
That allows push rights to our protected branches, first-reviewers rights, and getting your name on :ref:`about`.
There currently is not a clear process for getting to be a maintainer, but if you've been involved and contributing over a duration of several months, you may be a good candidate and should email the project lead listed on :ref:`about`.

Process
*******

After you've introduced yourself in a working group meeting (recommended, not required), you're ready to get started!
We recommend a typical open-source project flow and value detail and transparency.
If you commit to something and need to pull back, say so.
We all know priorities change and appreciate the heads up so that task can go into the open queue of tasks.

The process is simple and is as follow:

1. Create a ticket for any issues or features you'd like to see. You are not required to fix / implement patches required, but it would be helpful. Reporting bugs is also a valuable contribution.

2. If this ticket, or another existing ticket, is something you would like to work on, comment in the ticket claiming ownership over it. It would be helpful at this time if you declared a strategy and a timeline for planning purposes of other folks working around you. Over time, update the ticket with progress of key markers and engage in any constructive feedback maintainers or other users may have.

3. Once you've completed the task you set out to complete, submit a PR! Please fill out the PR template in complete to ensure that we have a full understanding of your work. At that point, 1-2 reviewers will take a look at your work and give it some feedback to be merged into the codebase. For trivial changes, a single maintainer may merge it after review if they're happy with it, up to their discretion. Any substantial changes should be approved by at least 1 maintainer and 1 other community member.

Note: We take code quality seriously and strive for high-quality and consistent code.
We make use of the linting and static analysis tools provided in ROS2 (``ament_cpplint``, ``ament_uncrustify``, ``ament_cppcheck``, etc).
All PRs are built in CI with the appropriate ROS distributions and run through a set of unit and system level tests including static analysis.
You can see the results of these tests in the pull request.
It is expected for feature development for tests to cover this work to be added.
If any documentation must be updated due to your changes, that should be included in your pull request.

Licensing
*********

Licensing is very important to open source projects. It helps ensure the
software continues to be available under the terms that the author
desired.

Because much of the source code is ported from other ROS 1 projects, each
package has it's own license. Contributions should be made under the predominant
license of that package. Entirely new packages should be made available under
the `Apache 2.0 license <https://www.apache.org/licenses/LICENSE-2.0>`_.

A license tells you what rights you have as a developer, as provided by
the copyright holder. It is important that the contributor fully
understands the licensing rights and agrees to them. Sometimes the
copyright holder isn't the contributor, such as when the contributor is
doing work on behalf of a company.

If for some reason Apache 2.0 or BSD licenses are not appropriate for your work, please get in contact with a project maintainer and discuss your concerns or requirements.
We may consider special exceptions for exceptional work, within reason (we will not accept any licenses that makes it unsuitable for commercial use).

.. _DCO:

Developer Certification of Origin (DCO)
***************************************

To make a good faith effort to ensure licensing criteria are met,
|LPN| encourages the Developer Certificate of Origin (DCO) process
to be followed.

The DCO is an attestation attached to every contribution made by a
developer. In the commit message of the contribution, (described more
fully later in this document), the developer simply adds a
``Signed-off-by`` statement and thereby agrees to the DCO.

In practice, its easier to just ``git commit -s -m "commit messsage."``.
Where ``-s`` adds this automatically.
If you forgot to add this to a commit, it is easy to append via: ``git commit --amend -s``.

When a developer submits a patch, it is a commitment that the
contributor has the right to submit the patch per the license.  The DCO
agreement is shown below and at http://developercertificate.org/.

.. code-block:: none

    Developer's Certificate of Origin 1.1

    By making a contribution to this project, I certify that:

    (a) The contribution was created in whole or in part by me and I
        have the right to submit it under the open source license
        indicated in the file; or

    (b) The contribution is based upon previous work that, to the
        best of my knowledge, is covered under an appropriate open
        source license and I have the right under that license to
        submit that work with modifications, whether created in whole
        or in part by me, under the same open source license (unless
        I am permitted to submit under a different license), as
        Indicated in the file; or

    (c) The contribution was provided directly to me by some other
        person who certified (a), (b) or (c) and I have not modified
        it.

    (d) I understand and agree that this project and the contribution
        are public and that a record of the contribution (including
        all personal information I submit with it, including my
        sign-off) is maintained indefinitely and may be redistributed
        consistent with this project or the open source license(s)
        involved.

